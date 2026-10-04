.PHONY: help backend-install backend-dev backend-test pipeline frontend-install frontend-dev frontend-build verify-meshes up down

help:
	@echo "Targets:"
	@echo "  backend-install   Create venv and install backend deps"
	@echo "  backend-dev       Run FastAPI (uvicorn --reload) on :8000"
	@echo "  backend-test      Run pytest"
	@echo "  pipeline          Run the end-to-end CAD->IK->JBI demo"
	@echo "  frontend-install  npm install"
	@echo "  frontend-dev      Run Next.js dev server on :3000"
	@echo "  frontend-build    Production build"
	@echo "  verify-meshes     SHA-256 check of vendored AR2010/D500 STLs"
	@echo "  up / down         docker compose up --build / down"

backend-install:
	cd backend && python -m venv .venv && . .venv/bin/activate && pip install -r requirements-dev.txt

backend-dev:
	cd backend && . .venv/bin/activate && uvicorn app.main:app --reload

backend-test:
	cd backend && . .venv/bin/activate && python -m pytest -q

pipeline:
	cd backend && . .venv/bin/activate && python pipeline_example.py

frontend-install:
	cd frontend && npm install

frontend-dev:
	cd frontend && npm run dev

frontend-build:
	cd frontend && npm run build

verify-meshes:
	python3 tools/verify_vendor_meshes.py

up:
	docker compose up --build

down:
	docker compose down
