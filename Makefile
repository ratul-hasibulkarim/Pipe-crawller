# PIPEBOT-X build. Every target is what CI runs; see .github/workflows/ci.yml.
#   make params   yaml -> bot_params.h / bot_params.json + protocol bindings (+ design calcs)
#   make cad      STEP/STL/PNG, interference check, BOM, mass report, assembly manual
#   make fw       RP2040 firmware for S/M/L (PlatformIO) + native unit tests
#   make sim      software-in-the-loop simulator binaries (build/sil/pipebot_sil_{S,M,L})
#   make pi       Pi companion: install deps + pytest (talks to the simulator)
#   make app      operator dashboard: npm ci + vitest + production build
#   make test     every unit test (Python, firmware native, app) + lint
#   make acceptance  T1-T15 on the simulator (QUICK=1 for the CI-length run)
#   make docs     BOM, mass report, pinmap, OpenAPI, acceptance report
#   make all
PY      ?= python3
NPM     ?= npm
JOBS    ?= $(shell nproc 2>/dev/null || echo 4)
SIZES   ?= S M L
QUICK   ?=
FW_ENVS := $(foreach s,$(SIZES),-e pico_$(s))

.PHONY: all params calcs cad fw fw-native fw-target sim pi app test lint acceptance docs clean

all: params cad fw sim pi app test acceptance docs

params:
	$(PY) tools/gen_params.py
	$(PY) tools/gen_protocol.py
	$(PY) tools/gen_pinmap_doc.py
	$(PY) calc/design_calcs.py

calcs:
	$(PY) calc/design_calcs.py

cad: params
	$(PY) cad/build.py --sizes $(SIZES)
	$(PY) tools/export_cad_params.py

fw: params fw-native fw-target

fw-native:
	cmake -S firmware/test -B firmware/build/test -DCMAKE_BUILD_TYPE=Debug
	cmake --build firmware/build/test -j$(JOBS)
	cd firmware/build/test && ctest --output-on-failure

fw-target:
	cd firmware && pio run $(FW_ENVS)

sim: params
	cmake -S firmware/sim_app -B build/sil -DCMAKE_BUILD_TYPE=Release
	cmake --build build/sil -j$(JOBS)

pi: sim
	$(PY) -m pip install -q -r pi/requirements.txt
	$(PY) -m pytest -q pi/tests

app:
	cd app && $(NPM) ci && $(NPM) test && $(NPM) run build

lint:
	$(PY) -m ruff check .
	$(PY) -m black --check .

test: lint params fw-native sim
	$(PY) -m pytest -q
	cd app && $(NPM) test

acceptance: sim
	$(PY) tests/acceptance/run_acceptance.py --sizes $(SIZES) $(if $(QUICK),--quick,)

docs: params
	$(PY) cad/build.py --sizes $(SIZES) --no-stl
	$(PY) pi/export_openapi.py
	$(PY) tests/acceptance/run_acceptance.py --sizes $(SIZES)

clean:
	rm -rf build firmware/build firmware/.pio app/dist cad/exports/*/stl tests/acceptance/out
