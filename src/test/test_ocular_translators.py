#!/usr/bin/python
"""
:author: Victor Gonzalez-Pacheco vgonzale@ing.uc3m.es
:date: 2015-02
"""

from __future__ import division
PKG = 'ocular_interaction'
import roslib
roslib.load_manifest(PKG)

import unittest
from itertools import chain
from ocular_interaction import ocular_atom_translators as otranslators
from dialog_manager_msgs.msg import AtomMsg, VarSlot
from ocular.msg import EventHandler


class TestOCULAREventHandlerTranslator(unittest.TestCase):

    def __init__(self, *args):
        super(TestOCULAREventHandlerTranslator, self).__init__(*args)

    def setUp(self):

        hand = 'left'
        event = 'learn'
        self.event_msg = EventHandler(hand=hand, event=event, last_event=event)

        self.def_slots = [VarSlot(name="type", val='event_handler'),
                          VarSlot(name="subtype", val='user'),
                          VarSlot(name="timestamp", val="_NO_VALUE_",
                                  type="number"),
                          VarSlot(name="consumed", val="false", type="string")]
        self.event_slots = \
            [VarSlot(name="hand", val=hand, type="string"),
             VarSlot(name="event", val=event, type="string"),
             VarSlot(name="last_event", val=event, type="string")]

        self.event_handler_atom = \
            AtomMsg(list(chain(self.def_slots, self.event_slots)))

    def tearDown(self):
        pass

    def test_generate_event_handler_slots(self):
        slots = list(
            otranslators.generate_event_handler_slots(self.event_msg))
        self.assertEqual(slots, list(self.event_slots))

    def test_event_handler_to_atom_empty_msg(self):
        self.assertEqual(AtomMsg(),
                         otranslators.event_handler_to_atom(None))

    def test_event_handler_to_atom(self):
        atom = otranslators.event_handler_to_atom(self.event_msg)
        self.assertEqual(self.event_handler_atom, atom)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_ocular_event_handler_translator',
                    TestOCULAREventHandlerTranslator)
