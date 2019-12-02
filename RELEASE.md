# Release Notes

Module excalibur-detector

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

