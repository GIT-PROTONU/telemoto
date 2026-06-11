# Vendored browser libraries (served at /static/ by ur_servo_controller.py)

| File | Source | Version |
|---|---|---|
| three.module.js | three `build/` | 0.169.0 |
| OrbitControls.js, STLLoader.js, ColladaLoader.js, TGALoader.js | three `examples/jsm/` | 0.169.0 |
| URDFLoader.js, URDFClasses.js | urdf-loader `src/` | 0.12.7 |

Local change: ColladaLoader.js line 42 — `'../loaders/TGALoader.js'` →
`'./TGALoader.js'` (everything is served flat from this directory). Bare
`'three'` / `'three/examples/jsm/loaders/*.js'` specifiers are resolved by the
importmap in the web page. viewer.js is ours, not vendored.
