# Lens profiles (fixtures)

Eleven phone lens profiles from Gyroflow's lens profile database,
https://github.com/gyroflow/lens_profiles, at commit
`886fc616c9c2c788856f54c21d451b9cb7fc4f44`, copied unchanged from its `Mobile phones/` folder.
The database is released under CC0 1.0 (public domain), by Gyroflow's contributors; the
`calibrated_by` field of each file names its calibrator.

`slam_it.eval.lenses` matches a video to one of them in the tests: the iPhone 14 Pro's and
the Pixel 8's main cameras, with a telephoto, an ultrawide, a lowercase brand, an "iPhone 14
Pro Max" that must not match an "iPhone 14 Pro", a calibration too rough to use (the Pixel 8's
1080p profile, 4 pixels RMS), a model that turns back before its image's corners (the
iPhone 11 Pro Max's wide add-on lens), and a profile named for the main camera whose own focal
length is an ultrawide's (the iPhone 12 Pro's at 30.00 fps, beside its "Normal" one).
