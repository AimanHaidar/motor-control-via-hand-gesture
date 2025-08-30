import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
import pickle

# 1. Load your CSV file
# Make sure it has a "target" column (True/False or 0/1)
data = pd.read_csv("data/thumb_points.csv")

# 2. Split features (X) and target (y)
X = data.drop(columns=["counted"])  # all columns except "counted"
y = data["counted"]                 # the target column

# 3. Split into train/test sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# 4. Train Random Forest
model = RandomForestClassifier(n_estimators=200, random_state=42)
model.fit(X_train, y_train)

# 5. Check accuracy
accuracy = model.score(X_test, y_test)
print(f"Accuracy: {accuracy:.2f}")

# 6. Save the model to a pickle file
with open("rf_model.pkl", "wb") as f:
    pickle.dump(model, f)
