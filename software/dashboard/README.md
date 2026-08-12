# Dashboard source — extraction pending

The existing Arm Lab and embedded Arm Chat run inside a broader private React
and Python application. The Arm components alone are not a standalone build:
they rely on the host entrypoint, global styles, test setup, backend routes,
session supervision, and other parent configuration.

A public dashboard release must provide:

- an arm-only frontend entrypoint and package/build files;
- an arm-only backend or a documented direct gateway interface;
- simulation that cannot be confused with live hardware;
- portable local authentication/configuration;
- focused UI/API tests in a fresh checkout;
- no bundled Codex/Arduino toolchains, sessions, tokens, or parent-app state.

Until that extraction is complete, this folder documents the boundary and must
not be described as runnable source.
