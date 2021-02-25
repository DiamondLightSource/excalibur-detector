# Release Notes

Module excalibur-detector

## [1-0-0beta] - 2020-11-06
### Added
- Dual 12 bit threshold checks

### Changed
- Merged master containing official 24bit release changes from STFC
- Improved logging
- Improved LV drop handling

## [0-11-0beta] - 2020-09-03
### Added
- Added dual 12 bit mode

### Changed
- Updated 24 bit mode to use fast firmware 24 bit implementation
- Improved connection logic using latest firmware driver features

## [0-10-5] - 2020-05-19
### Changed
- Update to odin-data 1-4-0dls3

## [0-10-4] - 2020-03-18
### Changed
- Update to odin-data 1-4-0dls2
- Update frameSimulator for integration testing

## [0-10-3] - 2020-02-12
### Changed
- Fixed acquisition bug that could drop the acquire parameter too early

## [0-10-2] - 2019-12-19
### Changed
- Update to odin-data 1-4-0dls1

## [0-10-1] - 2019-12-16
### Changed
- Improved log message to notify when the detector is armed
- Updated version in build file

## [0-10-0] - 2019-12-02
### Added
- Added RELEASE.md notes file
- Connection status monitoring (white records when detector not present)
- Error reporting on connection drop (status message human readable)
- DAC configuration reporting (logging and parameter access)
- Ensure calibration fails if there is no threshold0 file

### Changed
- Improved connection status reporting (status message human readable)
- Improved Low Voltage status reporting (status message human readable)

## [0-9-2] - 2019-10-01
### Changed
- Updated reference to odin-data 1-3-1dls1

## [0-9-1] - 2019-08-20
### Changed
- Fixed bug to ensure the correct trigger mode value is reported back to clients

