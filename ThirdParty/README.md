# Native dependencies

MuJoCo 4.0.0 is distributed from the official Windows x86-64 release:
https://github.com/google-deepmind/mujoco/releases/tag/4.0.0

Only the public headers, Win64 import library/runtime and upstream license/notices are included.
The runtime version is checked against the header version before use.

The private XML normalization implementation uses tinyxml2 10.0.0:
https://github.com/leethomason/tinyxml2/tree/10.0.0
The upstream source files retain their license notices.
