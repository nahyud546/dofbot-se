# t8_pipeline (LLM task pipeline)
Dời nguyên cụm từ `LargeModel_ws/` root (2026-09-28): `t8_*.py`, `state_service.py`,
`stub_planner.py`, `task_ontology.yaml`, `test_t8_pipeline.py`, `rag/`.
Giữ nguyên relative import (`from t8_pipeline import ...`, `from rag.retriever import ...`)
nên phải chạy với cwd hoặc PYTHONPATH = thư mục này:
`source scripts/setup/setup_env.sh && pytest projects/t8_pipeline/test_t8_pipeline.py`
