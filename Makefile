# Find all folders in assets/ that contain front.png
ASSETS_DIR := ./assets
RESULTS_DIR := ./results
FRONT_IMAGES := $(wildcard $(ASSETS_DIR)/*/front.png)

# Find all folders in assets/ that contain raw.mp4 for COLMAP
RAW_VIDEOS := $(wildcard $(ASSETS_DIR)/*/raw.mp4)

# Add output paths for other methods
# Define output paths for NextFace (append /NextFace to each folder name)
NEXTFACE_OUTPUTS := $(patsubst $(ASSETS_DIR)/%/front.png, $(RESULTS_DIR)/%/NextFace, $(FRONT_IMAGES))
GAUSSIAN_OUTPUTS := $(patsubst $(ASSETS_DIR)/%/front.png, $(RESULTS_DIR)/%/GaussianHaircut, $(FRONT_IMAGES))
HAIRSTEP_OUTPUTS := $(patsubst $(ASSETS_DIR)/%/front.png, $(RESULTS_DIR)/%/HairStep, $(FRONT_IMAGES))
SUGAR_OUTPUTS := $(patsubst $(ASSETS_DIR)/%/raw.mp4, $(RESULTS_DIR)/%/sugar, $(RAW_VIDEOS))

# Default GPU ID (can be overridden from command line)
GPU ?= 0

# print nextface outputs
$(info $(ASSETS_DIR))
$(info $(NEXTFACE_OUTPUTS))

# Path to Conda installation (adjust if different)
CONDA_PATH := /home/pkw/miniconda3

# Update all target
all: nextFace gaussianHaircut hairStep sugar

# NextFace target
.PHONY: nextFace
nextFace: $(NEXTFACE_OUTPUTS)

# Rule to process each front.png with NextFace using the shell script
$(RESULTS_DIR)/%/NextFace: $(ASSETS_DIR)/%/front.png
	@echo "Processing $< for NextFace..."
	@mkdir -p $@
	@./nextFace.sh $< $@

# GaussianHaircut target
.PHONY: gaussianHaircut
gaussianHaircut: $(GAUSSIAN_OUTPUTS)

$(RESULTS_DIR)/%/GaussianHaircut: $(ASSETS_DIR)/%/front.png
	@echo "Processing $< for GaussianHaircut..."
	@mkdir -p $@
	@./gaussianHaircut.sh $< $@ $(GPU)

# HairStep target
.PHONY: hairStep
hairStep: $(HAIRSTEP_OUTPUTS)

$(RESULTS_DIR)/%/HairStep: $(ASSETS_DIR)/%/front.png
	@echo "Processing $< for HairStep..."
	@mkdir -p $@
	@./hairstep.sh $< $@

# SuGaR target
.PHONY: sugar
sugar: $(SUGAR_OUTPUTS)

# Rule to process each raw.mp4 with SuGaR using a shell script and compress the database.db file
$(RESULTS_DIR)/%/sugar: $(ASSETS_DIR)/%/raw.mp4 | $(RESULTS_DIR)/%/sugar
	@echo "Processing $< for SuGaR..."
	@mkdir -p $(@D)
	@./sugar_process.sh $< $@ $@
	@echo "Compressing database.db..."
	@cd $@ && tar -czf database.tar.gz database.db

# Ensure frame extraction directories exist
$(RESULTS_DIR)/%/sugar:
	@mkdir -p $@

convert:
	@bash convert_files.sh

# Update help
help:
	@echo "Usage:"
	@echo "  make nextFace        - Process all front.png with NextFace"
	@echo "  make gaussianHaircut - Process all front.png with GaussianHaircut"
	@echo "  make hairStep        - Process all front.png with HairStep"
	@echo "  make all             - Run all methods"
	@echo "  make clean           - Remove all results"
	@echo "  make help            - Show this help message"