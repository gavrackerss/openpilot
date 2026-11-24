# UI restoration — 0.11.2

Baseline: c62e642 (0.11.1). Current production parent: e1a98f3 (0.11.2).
This review branch restores Tesla, Models and OSM navigation, the original
offroad speed-sign preview, and saved parameter visibility. The existing
native offroad model preference selector is available under Models.
Menu sizing adapts to panel height.

This is not a complete UI or vehicle feature port. Tesla settings are
read-only, model selection retains native fallback, OSM is status-only
because mapd is not present in the 0.11.2 tree, and onroad speed-set
buttons remain absent. Auto-resume and independent lateral engagement
during pedal override remain offline reference behavior. No changes to
controlsd, selfdrived, Panda, planner, runtime model approval or safety
settings are made. DEC and LS2 remain out of scope.
