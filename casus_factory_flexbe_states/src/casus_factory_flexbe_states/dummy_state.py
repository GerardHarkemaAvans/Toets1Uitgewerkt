#!/usr/bin/env python
from flexbe_core import EventState, Logger


class DummyState(EventState):
    '''
    Checks if the given condition is pass and returns the corresponding outcome.
    This state can be used if the further control flow of the behavior depends on a simple condition.

    -- active_output    bool	True activates pass output, False the activates failed output


    <= pass 					Returned if active_output == True
    <= failed 					Returned if active_output == False
    '''

    def __init__(self, active_output):
        super(DummyState, self).__init__(outcomes=['pass', 'failed'])
        self._active_output = active_output
        self._outcome = 'failed'

    def execute(self, userdata):
        return self._outcome

    def on_enter(self, userdata):
            self._outcome = 'pass' if self._active_output else 'failed'

