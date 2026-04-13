from sklearn.linear_model import LinearRegression
import numpy as np
import matplotlib.pyplot as plt

X = np.array(distance).reshape(-1, 1)
y = np.array(rpm)

model = LinearRegression()
model.fit(X, y)

slope = model.coef_[0]
intercept = model.intercept_
r2 = model.score(X, y)

print(f"Slope:     {slope:.4f}")
print(f"Intercept: {intercept:.4f}")
print(f"R² Score:  {r2:.4f}")

# Plot
X_line = np.linspace(X.min(), X.max(), 300).reshape(-1, 1)
y_line = model.predict(X_line)

plt.figure(figsize=(8, 5))
plt.scatter(X, y, color='red', zorder=5, label='Data')
plt.plot(X_line, y_line, color='blue', label=f'Fit: RPM = {slope:.2f}x + {intercept:.2f}')
plt.xlabel('Distance')
plt.ylabel('RPM')
plt.title('Distance vs RPM - Linear Regression')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
