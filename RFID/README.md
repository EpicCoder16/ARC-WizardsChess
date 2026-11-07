# Task 4: RFID positioning and centering

This is a prototype for the Pico 2 W described in the handoff. `pico_readers.py` adapts the separately supplied `two_reader_test.py` pin configuration, alternating antenna scans, and minimum-gain measurement. That test detects a card but does not read its UID. The EpicCoder16 repository does not contain Task 3's motion API. **UID acquisition and motion integration are still required; this module has not been tested on a robot.** Do not operate motors until the adapters and limits have been reviewed and tested on the actual robot.

## Interfaces

Create `RFIDPosition(readers, motion, tiles, ...)`. A tile map associates normalized hex UID strings with `(square, x_mm, y_mm)` in the agreed board coordinate system. Coordinates are explicit: do not assume a board origin or orientation. The 610 mm tile width in the handoff is approximate; measure and agree on coordinates with Task 3.

* `current_tile()` returns the mapped square or `None` when no mapped card is seen. Conflicting UIDs raise `CenteringError`.
* `center_on_card()` finds the observed card's lateral gain balance, scans both longitudinal detection edges, moves to their midpoint and rechecks lateral balance. It returns the square and corrects the pose only on success. It raises `CenteringError` for lost, conflicting, unknown, or uncenterable cards and always calls `motion.stop()`.
* `card_offset()` returns `None` until a successful scan, then a dictionary with `forward_mm_from_scan_start` and final `gain_difference`. Gain difference is not a calibrated lateral distance; the scan start is after lateral alignment, and this value is not a live position measurement.
* `search_for_card(radius_steps=3)` performs a bounded square spiral in the robot frame and returns the first mapped square or `None`. It does not automatically center the robot. Ensure the entire search area is clear before calling it.

`tile_map.register(tiles, uid, square, x_mm, y_mm)` checks duplicate UIDs and squares. Use `tile_map.save(tiles, path)` and `tile_map.load(path)` for JSON persistence. Obtain each UID from the actual reader, verify the square and measured coordinates, and register one card at a time. Back up the mapping before replacing it; filesystem rename behavior varies on MicroPython and some ports do not replace an existing destination.

## Adapter contract

* `readers.read_pair()` returns `(left, right)`, each either `None` or `(uid, minimum_gain)`; UID is bytes or a hex string and gain is a comparable number. `PicoReaders(read_uid)` implements the supplied gain scan and requires a `read_uid(reader)` callback that performs RC522 anticollision/selection and returns a UID or `None`. The callback must operate only the reader passed to it and turn its antenna off even on failure. `PicoReaders` switches both antennas off between samples and on exit, and refuses to invent a UID if acquisition fails. Verify the card is still in place across gain and UID reads. Confirm that a lower reported value actually means closer; raw register encodings may differ.
* `motion.move_relative(right_mm, forward_mm)` is a **blocking**, robot-frame move in millimeters; positive values mean robot-right and robot-forward. It must enforce its own timeout, travel limits and emergency stop, and raise on motion failure. Verify signs and wheel geometry with Task 3. Do not substitute the existing `move_to` without an adapter.
* `motion.get_pose()` returns `(x_mm, y_mm, heading)`; `motion.set_pose(x_mm, y_mm, heading)` accepts the same heading units. `motion.stop()` must halt motion even after a failed operation.

For a visible detection, the robot can be anywhere in the reader's field; immediately assigning the tile-center pose would introduce an unknown error. This prototype applies exact tile coordinates only after successful centering. If Task 3 needs corrections while crossing a card, agree on a calibrated offset-aware correction with that team.

## Bring-up and limitations

1. The supplied reader test configures SPI SCK GP18, MOSI GP19, MISO GP16, reader selects GP20 and GP17, and shared reset GP22. Confirm actual wiring and 3.3 V supply with the team. `PicoReaders` uses those pins and checks the reader version registers.
2. Implement the missing UID-reading callback and bench-test the reader adapter with motors disconnected. Verify antenna sequencing, UID format, gain ordering, reader identity, and repeatability at known offsets. A tie in discrete gain levels is not proof of millimeter-level centering.
3. Agree on coordinates, axis signs, stop behavior, safe speed, timeout and movement resolution with Task 3. Implement the motion adapter. Validate travel limits and emergency stop with wheels raised before floor testing.
4. Measure detection boundaries across the full 30 mm starting-offset range. The scan needs an initial detection, both readers must detect for lateral alignment, and each edge must be within `max_travel_mm`. Reader field asymmetry, quantized gain, tag orientation and hysteresis can bias the midpoint. Calibrate step size, gain tolerance, and scan direction; a successful return is not a guarantee of accuracy.
5. Test on foam with a clear area and a person ready to stop the robot. Record errors for multiple cards, headings, and starting offsets up to 30 mm. The handoff's few-millimeter requirement is not demonstrated by the simulated tests.

The module requires the `RFID` folder on the MicroPython import path. Check supported MicroPython features and filesystem behavior on the installed firmware before deployment.

## PC simulation tests

From the repository root, run `python -m unittest discover -s tests -v`. The tests use a simplified symmetric simulated field, not an RC522 or real motors; they verify control flow and error handling only.
