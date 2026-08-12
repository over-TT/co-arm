# Raspberry Pi gateway source — pending

This folder is reserved for the complete clean `robot_gateway` Python package,
its tests, dependency pins, and portable service/configuration templates. The
package must be copied as a unit because its runtime spans API, state,
calibration, simulation, camera, serial controller, physical commissioning,
and simple-arm modules.

Public configuration must accept endpoint, credential-file, state-directory,
camera, and UART choices locally. It must not encode private addresses, local
paths, tokens, known-hosts data, controller IDs, or retained camera frames.

Picamera2/libcamera should be documented as Raspberry Pi OS packages rather
than silently vendored. Source will be added after scope and licensing are
confirmed and a fresh-checkout gateway test run passes.
