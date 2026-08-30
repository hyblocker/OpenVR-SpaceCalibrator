# Translating

The ground truth translation will always be available at `assets/en_GB.json`. If you notice that something is different there when compared to the language you're translating please use the `en_GB` file as reference.

Translations are authored in JSON. They're a key value pair of IDs to the text:

```json
{
  "app_title": "{0} {1}", /* 0 : Space Calibrator ; 1 : version (eg 2.0.0) */
  "app_title_vr": "{0} {1} - close VR overlay to use mouse", /* 0 : Space Calibrator ; 1 : version (eg 2.0.0) */
  "tracking_system_no_systems": "No tracked devices present. Please turn on a device to continue.", /* Should never be hit but you never know */

  "ipc_unavailable": "Failed to connect to Space Calibrator SteamVR driver. Please verify file integrity in Properties -> Installed Files, or re-install the app.",
  "steamvr_unavailable": "Failed to connect to SteamVR.",
  "vr_troubleshooting_hmd_not_found": "Please connect your VR headset and launch SteamVR first.",
  "vr_troubleshooting_connect_steamlink": "Please open SteamLink on your VR headset and connect to SteamVR first.",
  "vr_troubleshooting_generic": "Please connect your VR headset and launch SteamVR first. ({0} {1})", /* 0 : SteamVR init error (eg 108), 1 : SteamVR init error string (eg Headset not connected) - this string is provided by Steam. */

  // space here refers to a tracking space / tracking system.
  "reference_space": "Reference space",
  "target_space": "Target space",
  "select_reference_device": "Select reference device",
  "select_target_device": "Select target device"
}
```

The format supports comments, which are usually used to inform you of what numbered arguments are.

Text formatting may use `{N}`, where `N` is a numbered argument so that you can re-order text to make grammatical sense in your language of choice.

For example, you can write: `Arg0: {0} arg1: {1}` ; and itll appear as `Arg0: foo arg1: bar` or `Arg0: {1} arg1: {0}` which appears as `Arg0: bar arg1: foo`.

You can experiment with translations by making a `json` file matching a locale string at `<SPACECAL-DIR>/assets/lang/en_GB.json`

Missing strings will fall back to their respective English strings. For example, if you are editing `it.json` and forgot to add the string `reference_space`, the app will show the English value to the user.

You should also translate the languages block at the bottom of the file. This section is used to present the name of the language to the user in the language selection dropdown in the current language and the language itself. For example, if the user currently has Japanese selected, the entry for French would read as `フランス語 (Français)` in the dropdown.

The supported languages right now are:
- `en_GB.json` (maintained by me, consider it ground truth)
- `en_US.json` English (US)
- `fr.json` French
- `it.json` Italian
- `de.json` German
- `nl.json` Dutch (missing)
- `es_ES.json` Spanish (Spain)
- `es_US.json` Spanish (Latin America) (missing)
- `da.json` Danish (missing)
- `sv.json` Swedish (missing)
- `fi.json` Finnish (missing)
- `no.json` Norwegian
- `bg.json` Bulgarian (missing)
- `pl.json` Polish
- `cs.json` Czech (missing)
- `el.json` Greek (missing)
- `hu.json` Hungarian
- `pt_PT.json` Portuguese (Portugal) (missing)
- `pt_BR.json` Portuguese (Brazil) (missing)
- `ro.json` Romanian (missing)
- `ru.json` Russian
- `tr.json` Turkish (missing)
- `uk.json` Ukrainian (missing)
- `zh_HANS.json` Chinese (Simplified)
- `zh_HANT.json` Chinese (Traditional) (missing)
- `ja.json` Japanese
- `ko.json` Korean
- `th.json` Thai (missing)
- `vi.json` Vietnamese (missing)

These files must live at `<SPACECAL-DIR>/assets/lang/<LANGUAGE>.json`.

Languages with `(missing)` have not been translated yet. If you know the language, feel free to make a translation. Note that machine generated translations (eg AI, Google Translate) will be rejected indiscriminately.
