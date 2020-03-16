# Introduction

This package contains nodes that execute and advertise vision actions, e.g.:

- Part recognition
- Tooltip alignment
- Visual servoing
- Orientation checks

All vision skills should be action-based, so that calculations are allowed to fail and time out.

For this, the Python nodes should advertise a number of actions, which are defined in o2ac_msgs.