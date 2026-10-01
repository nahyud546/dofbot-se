import re

with open('projects/vision_experiments/identify_cube.py', 'r') as f:
    content = f.read()

content = content.replace('DB_PATH_TMPL = "ai/datasets/trash-images/processed/vector_database_{}.pt"', 
                          'DB_PATH_TMPL = __file__.split("projects")[0] + "ai/datasets/trash-images/processed/vector_database_{}.pt"')

with open('projects/vision_experiments/identify_cube.py', 'w') as f:
    f.write(content)
