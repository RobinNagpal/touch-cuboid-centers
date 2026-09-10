# assemble-table — one entry point for everything.
#
# Run `make` on its own to see what is available.

SHELL := /bin/bash
.DEFAULT_GOAL := help

SEED ?= 1
GUI  ?= true
RVIZ ?= false

# Run a command inside the pixi environment with the workspace overlay sourced.
ros = pixi run bash -c 'source install/setup.bash && $(1)'

.PHONY: help setup build run cell test lint clean doctor

help: ## Show this help
	@echo ""
	@awk 'BEGIN {FS = ":.*##"} \
		/^##@/ { printf "\n  \033[1m%s\033[0m\n", substr($$0, 5) } \
		/^[a-zA-Z_.-]+:.*?##/ { printf "    \033[36m%-8s\033[0m %s\n", $$1, $$2 }' $(MAKEFILE_LIST)
	@echo ""
	@echo "  Settings: SEED=$(SEED) GUI=$(GUI) RVIZ=$(RVIZ)"
	@echo "  For example: make run SEED=7"
	@echo ""

##@ Running

run: build ## Start the cell and assemble the table (this is the one command)
	$(call ros,ros2 launch table_assembly run.launch.py \
		seed:=$(SEED) gui:=$(GUI) rviz:=$(RVIZ))

cell: build ## Start the cell but leave the arm alone, for poking at by hand
	$(call ros,ros2 launch table_assembly run.launch.py task:=false \
		seed:=$(SEED) gui:=$(GUI) rviz:=$(RVIZ))

##@ Project

setup: ## Install the environment (the first run downloads a few gigabytes)
	pixi install

build: setup ## Build the workspace
	@pixi run colcon build --symlink-install

test: build ## Run the tests
	$(call ros,pytest src/table_assembly/test -q)

lint: setup ## Check code style
	pixi run ruff check src/
	pixi run ruff format --check src/

clean: ## Remove build/, install/ and log/
	pixi run clean

doctor: ## Print versions of everything that matters
	@printf "%-14s %s\n" "pixi" "$$(pixi --version 2>&1)"
	@pixi run bash -c 'printf "%-14s %s\n" "python" "$$(python --version 2>&1)"; \
		printf "%-14s %s\n" "ROS distro" "$$ROS_DISTRO"; \
		printf "%-14s %s\n" "Gazebo" "$$(gz sim --versions 2>&1 | head -1)"'
