.PHONY: setup build check publish-test publish clean

DIST_DIR := dist

setup:
	uv pip install -e .

build:
	uv run --no-project --with build python -m build

check:
	uv run --no-project --with twine twine check $(DIST_DIR)/*

publish-test:
	uv run --no-project --with twine twine upload --repository testpypi $(DIST_DIR)/*

publish:
	uv run --no-project --with twine twine upload $(DIST_DIR)/*

clean:
	git clean -fdX .
