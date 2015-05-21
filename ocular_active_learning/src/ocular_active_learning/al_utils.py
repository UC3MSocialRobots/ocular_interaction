#!/usr/bin/env python
# -*- coding: utf-8 -*-

# :license       LASR_UC3M v1.0, ver LICENCIA.txt

# Este programa es software libre: puede redistribuirlo y/o modificarlo
# bajo los terminos de la Licencia Academica Social Robotics Lab - UC3M
# publicada por la Universidad Carlos III de Madrid, tanto en su version 1.0
# como en una version posterior.

# Este programa se distribuye con la intencion de que sea util,
# pero SIN NINGUNA GARANTIA. Para mas detalles, consulte la
# Licencia Academica Social Robotics Lab - UC3M version 1.0 o posterior.

# Usted ha recibido una copia de la Licencia Academica Social
# Robotics Lab - UC3M en el fichero LICENCIA.txt, que tambien se encuentra
# disponible en <URL a la LASR_UC3Mv1.0>.
"""
Utilities for OCULAR's Active Learning Nodes.

:author: Victor Gonzalez-Pacheco
"""
# from functools import partial
from itertools import chain, islice
# from toolz import (compose, frequencies, take)
# from toolz import (compose, frequencies)
import numpy as np
import nltk
import toolz as tz
import pandas as pd

import roslib
roslib.load_manifest('ocular_active_learning')
# import rospy

# Calc frequencies of a list and return them as series
frequencies = tz.compose(pd.Series, tz.frequencies)


class Accumulator(object):

    """
    A List Container with max length set up in maxlen parameter.

    If new element added and list is full, then the prev. elems are flushed
    """

    def __init__(self, maxlen, seq=None):
        """
        Constructor.

        Args:
            :maxlen: Max num of elements of the accumulator.
            :seq: sequence to initialize the Accumulator. Default=None

        """
        self.maxlen = maxlen
        self.l = []     # Need no initialize
        self.insert(seq)

    def append(self, item):
        """
        Add a new item to the list.

        If len(list) >= maxlen, it flushes the list and then adds the item

        Example:

            >>> acc = Accumulator(3)
            >>> acc.append(1)   # Also acc(1)
            [1]
            >>> acc.append(2)
            [1, 2]
            >>> acc.append(3)
            [1, 2, 3]
            >>> acc.get()       # Also acc()
            [1, 2, 3]
            >>> acc.append(4)
            [4]
            >>> acc.get()
            [4]

        :see: Accumulator.__call__
        """
        if len(self.l) == self.maxlen:
            self.clear()
        self.l.append(item)
        return self

    def insert(self, seq):
        """
        Insert a sequence to the list.

        Note:
            if len(self.l+seq) >= maxlen only keeps the last maxlen items

        >>> acc = Accumulator(3)
            >>> acc.insert([1, 2])
            [1, 2]
            >>> acc.insert([3, 4])
            [4]
            >>> acc.insert([5, 6])
            [4, 5, 6]
            >>> acc.insert([7, 8, 9])
            [7, 8, 9]
            >>> acc.insert([10])
            [10]
            >>> acc.insert([11])
            [10, 11]
            >>> acc.insert([])
            [10, 11]
        """
        if not seq:
            return self

        for item in seq:
            self.append(item)
        # total_len = len(self.l) + len(seq)
        # start = (total_len // self.maxlen) * self.maxlen
        # self.l = list(islice(chain(self.l, seq), start, total_len))
        return self

    def get(self):
        """Return the list. Equivalent to acc."""
        return self.l

    def isfull(self):
        """Return True if Accumulator is full. Otherwise return False."""
        return len(self.l) >= self.maxlen

    def isempty(self):
        """Return True if Accumulator is empty. Otherwise return False."""
        return not bool(self)

    def haselems(self):
        """Return True if Accumulator has any elem. Otherwise return False."""
        return not self.isempty()

    def clear(self):
        """Clear accumulator's list.

        Example:

            >>> acc = Accumulator(3)
            >>> acc(1)
            [1]
            >>> acc(2)
            [1, 2]
            >>> acc.clear()
            >>> acc()
            []
        """
        del self.l[:]

    def __call__(self, item=None):
        """
        Add an element if item != None else returns the list.

        Example:

            >>> acc = Accumulator(3)
            >>> acc(1)
            [1]
            >>> acc(2)
            [1, 2]
            >>> acc()
            [1, 2]
        """
        if item:
            return self.append(item)
        else:
            return self.get()

    def __nonzero__(self):
        """Return True if list has items, False otherwise.

        Example:

            >>> acc = Accumulator(3)
            >>> bool(acc)
            False
            >>> acc(1)
            [1]
            >>> bool(acc)
            True
            >>> acc.insert([2,3])
            [1, 2, 3]
            >>> bool(acc)
            True
            >>> acc(4)
            [4]
            >>> bool(acc)
            True
        """
        return bool(self.l)

    def __repr__(self):
        """Represent the object as string."""
        return str(self.l)

    def __iter__(self):
        """Magic method to implement Iterator protocol."""
        for x in self.l:
            yield x

    def __len__(self):
        """Return the len of the contained list.

        Example:

            >>> acc = Accumulator(3)
            >>> len(acc)
            0
            >>> acc(1)
            [1]
            >>> len(acc)
            1
            >>> acc.insert([2,3])
            [1, 2, 3]
            >>> len(acc)
            3
            >>> acc(4)
            [4]
            >>> len(acc)
            1
        """
        return len(self.l)


def estimate(predictions_rgb, predictions_pcloud, weights=(0.6, 0.4)):
    """
    Return ids of object that appears more times in each matcher.

    That is, returns the id of the item with higher frequency in
    the rgb predictions, the point cloud predictions and
    the item with higer frequency in a weighted sum of rgb and pcloud preds.

    Args:
        predictions_rgb: list of predictions of the RGB matcher
        predictions_pcloud: list of predictions of the point cloud matcher
        weights: tuple to indicate relative weights of the matchers.
            Default=(0.6, 0.4)

    Return:
        tuple with (id_of_weighted_sum, id_rgb, id_pcloud)

    Example:

        >>> estimate([1, 1, 1, 2, 2], [1, 0, 2, 2, 1])
        (1, 1, 1)
        >>> estimate([1, 1, 1, 2, 2], [1, 0, 2, 2, 2])
        (2, 1, 2)
        >>> estimate([1, 1, 1, 2, 2], [1, 0, 0, 0, 2])
        (1, 1, 0)
        >>> estimate([1, 1, 1, 2, 2], [2, 2, 2, 2, 2], weights=(1,0))
        (1, 1, 2)
    """
    w_rgb, w_pcloud = weights
    freqs_rgb = frequencies(predictions_rgb)
    freqs_pcloud = frequencies(predictions_pcloud)
    freqs = pd.Series.add(w_rgb * freqs_rgb,
                          w_pcloud * freqs_pcloud, fill_value=0)
    return (freqs.nlargest(1).index[0],
            freqs_rgb.nlargest(1).index[0],
            freqs_pcloud.nlargest(1).index[0])


def margin(items):
    """
    Calculate margin between highest two frequencies.

    The margin is the difference between the frequency of appearance between
    the two most common items in a list.

    Example:

        >>> margin([3, 3, 3, 1, 2, 0])
        2.0
        >>> margin([2, 2, 2, 1, 1])
        1.0
        >>> margin([3, 3, 5, 5, 6])
        -0.0
        >>> margin([3, 3, 3, 3, 3])
        nan
        >>> margin([1, 1, 1, 1, 2, 2, 3, 4, 5, 0, 0])
        2.0
    """
    freqs = frequencies(items)
    return freqs.nlargest(2).diff().values[-1] * (-1)


def entropy(labels):
    """
    Return entropy of a list of categorical variables.

    Example:

        >>> entropy(['red', 'red', 'green', 'red'])
        0.81127812445913283
        >>> entropy(['red', 'red', 'red', 'green', 'green', 'blue'])
        1.4591479170272448
    """
    freqdist = nltk.FreqDist(labels)
    probs = [freqdist.freq(l) for l in nltk.FreqDist(labels)]
    probs = np.array(probs)
    return -sum(probs.T * np.log2(probs))


def numerize(array):
    """
    Return an int-based id for each categorical variable of the passed list.

    Args:
    :array (list of str): The list of categorical variables to numerize

    Return:
    numerized (tuple(int, str)): Returs a lit of tuples where each tuple is
        the numeric id for each categorical variable and the variable itself.

    Example:

        >>> numerize('red green blue white'.split(' '))
        [(0, 'blue'), (1, 'green'), (2, 'red'), (3, 'white')]

    Note that repeated variables are numreized only once:

        >>> numerize('red green blue white blue white'.split(' '))
        [(0, 'blue'), (1, 'green'), (2, 'red'), (3, 'white')]
    """
    return list(enumerate(sorted(set(array))))
