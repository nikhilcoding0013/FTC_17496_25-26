import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

X = np.array(distance)
y = np.array(servo)

def model(x, d, a, b, c):
    return d + a * b**(x - c)

def loss(params):
    d, a, b, c = params
    y_pred = model(X, d, a, b, c)
    return float(np.sum((y - y_pred) ** 2))

# d > 0, a < 0, 0 < b < 1, c > 0
result = minimize(loss, x0=[0.65, -0.65, 0.95, 30.0], method='Nelder-Mead')
d, a, b, c = result.x

y_pred = model(X, d, a, b, c)
r2 = 1 - np.sum((y - y_pred)**2) / np.sum((y - np.mean(y))**2)

print(f"d: {d:.4f}")
print(f"a: {a:.4f}")
print(f"b: {b:.4f}")
print(f"c: {c:.4f}")
print(f"R² Score: {r2:.4f}")
print(f"\nModel: servo = {d:.4f} + ({a:.4f}) * {b:.4f}^(distance - {c:.4f})")

X_line = np.linspace(0, 170, 300)
plt.scatter(X, y, color='red', label='Data')
plt.plot(X_line, model(X_line, d, a, b, c), color='blue', label='Fit: ServoPos = 0.651 + (-0.669) * 0.975^(x - 31.006)')
plt.title('Distance vs Servo Position - Exponential Regression')
plt.xlabel('Distance')
plt.ylabel('Servo')
plt.legend()
plt.show()
