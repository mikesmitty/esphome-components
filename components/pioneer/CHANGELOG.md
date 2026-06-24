# Changelog

## [0.4.3](https://github.com/mikesmitty/esphome-components/compare/pioneer-v0.4.2...pioneer-v0.4.3) (2026-06-24)


### Bug Fixes

* **pioneer:** add synchronous parameter to actions ([b4827f7](https://github.com/mikesmitty/esphome-components/commit/b4827f78f22d67236e4eb70c232c943bb14f5c67))
* **pioneer:** enable follow_me flag on IR remote temp transmission ([fe765f5](https://github.com/mikesmitty/esphome-components/commit/fe765f564503bc0e6e7068356948f109fdf1d843))
* **pioneer:** fix startup fan mode initialization ([ba2fe8d](https://github.com/mikesmitty/esphome-components/commit/ba2fe8dbf9c174b52274b7cf07e53a5d7cb4789a))

## [0.4.2](https://github.com/mikesmitty/esphome-components/compare/pioneer-v0.4.1...pioneer-v0.4.2) (2026-06-17)


### Bug Fixes

* **pioneer:** support standard fan modes alongside custom fan modes ([fb3248e](https://github.com/mikesmitty/esphome-components/commit/fb3248e8defeb34bcedceb4e5d0abb64ad986d6c))

## [0.4.1](https://github.com/mikesmitty/esphome-components/compare/pioneer-v0.4.0...pioneer-v0.4.1) (2026-04-25)


### Bug Fixes

* **pioneer:** migrate custom fan modes to Climate base class setter ([7d35783](https://github.com/mikesmitty/esphome-components/commit/7d35783118b081ccc3924278748a9f701821d9ec)), closes [#8](https://github.com/mikesmitty/esphome-components/issues/8)

## [0.4.0](https://github.com/mikesmitty/esphome-components/compare/pioneer-v0.3.0...pioneer-v0.4.0) (2026-04-25)


### Features

* add action support and fix various bugs ([1d3457b](https://github.com/mikesmitty/esphome-components/commit/1d3457b0cb6f4bbc32fb159a2f026018154bf511))
* finish follow me/remote temp implementation ([e4f21a2](https://github.com/mikesmitty/esphome-components/commit/e4f21a28b625d1fa8f0a21c910dfeffaeed707e0))
* initial pioneer wyt esphome module ([d8e6d38](https://github.com/mikesmitty/esphome-components/commit/d8e6d38781e6d46e282377ab9975b05ae4593f2a))


### Bug Fixes

* auto-load binary_sensor component ([33a4ba6](https://github.com/mikesmitty/esphome-components/commit/33a4ba6086e8d99eab2032e8ee26aa4bd8d8f415))
* delay publishing updates after commands ([423f2c7](https://github.com/mikesmitty/esphome-components/commit/423f2c75d561d6acc3b109ea69c0aaea9b70567f))
* handle state reversion during updates ([7986dae](https://github.com/mikesmitty/esphome-components/commit/7986dae1ddfb589ea43df7cc93180ad63d1ecf81))
* make remote ifdef functional ([d806e6d](https://github.com/mikesmitty/esphome-components/commit/d806e6d8fc34495e5656a9201b545b19890f3ad7))
* make sure fan modes actually show as changed ([#5](https://github.com/mikesmitty/esphome-components/issues/5)) ([7aedbf4](https://github.com/mikesmitty/esphome-components/commit/7aedbf489e01c7073966e767e6548d4e653047b6))
* **pioneer:** correct climate state reporting to Home Assistant ([#9](https://github.com/mikesmitty/esphome-components/issues/9)) ([93f2a6a](https://github.com/mikesmitty/esphome-components/commit/93f2a6abbdd0215c128ce2c07b1697ad346cae29))
* pull display bool from config not state ([b334528](https://github.com/mikesmitty/esphome-components/commit/b334528acf6d63332cdc110707243d4e65464c19))
* switch to new fan mode setters for 2025.11 ([#3](https://github.com/mikesmitty/esphome-components/issues/3)) ([7080527](https://github.com/mikesmitty/esphome-components/commit/708052756efcb34f2cbd770b8f9b090336336025))
* update to use climate_schema(...) ([5dc9735](https://github.com/mikesmitty/esphome-components/commit/5dc9735483d82db4eadbfad32537f8d9621a5740))
