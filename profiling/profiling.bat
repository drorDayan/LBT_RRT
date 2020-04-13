cd ..
venv\Scripts\python.exe -m cProfile -s cumtime polygons_scene.py >profiling\temp
cd profiling