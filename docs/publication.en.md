[Français](publication.md) · **English**

# Provenance of this edition

This edition is a selected snapshot of the FieldMind working repository. The first edition (21 October 2026) took the working tree, local changes included. The 23 October 2026 update takes the rover software, the ESP32-C5 firmware and the printable CAD from a commit of the source repository, without uncommitted local changes. A safety test whose obstacle fell exactly on the absolute stopping distance (a result that depended on floating-point rounding under Linux) was moved closer, to 0.62 m. It does not include the history of the private working repository.

The [`snapshot-source.json`](snapshot-source.json) manifest gives the reference source commit, the capture time and the hashes of the source files copied before the publication adaptations. The source hashes are therefore not those of all published files: the network parameters, the AI profile of the quick start, the CAD paths and the documentation were adapted in this edition.

## Selection

Kept: current rover software, C5 firmware, tests and ROS 2 bridge, first web simulation, deployment code, current printable parts, drawings, Fusion scripts, A7 sensor board and useful KiCad studies.

Left out: machine and account access, keys, Challenge emails, orders and delivery addresses, operator data, raw logs, Python environments, ESP builds, replaced former supervision, video captures, raw footage, `.blend` files, old CAD trials and external geometry with unverified rights. Existing publications and services are not redeployed by this repository.

Also left out on 23 October: detailed analysis notes on the drone app, internal work report, GX10 gateway and tunnel configuration, a CAD render containing a third-party servo model. Adaptations specific to this edition: rover and network addresses replaced by examples, the name of a home network removed from the firmware's list of ignored networks, and the default photo folder corrected (`mission/executeur.py`) so that the tests do not write to the user's folder.

The quick start enforces the `aucun` AI profile, the network addresses are examples and the GX10 services bind to the loopback interface by default. Using it from a rover requires explicitly configuring the appropriate address, key and private network.

## Images and video

The CAD view in the README comes from the project; it is not a photograph of an assembled rover. Future photos, test videos and measured demonstrations may complement the README and the portfolio. No placeholder link is shown in their place.
