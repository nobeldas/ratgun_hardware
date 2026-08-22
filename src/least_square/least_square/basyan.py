import numpy as np
import matplotlib.pyplot as plt

# Data
X = np.array([[1,1],[2,1],[3,1]])
y = np.array([2,3,5])

# Prior strength
lambda_ = 1

# Compute posterior
Sigma = np.linalg.inv(X.T @ X + lambda_ * np.eye(2))
mu = Sigma @ X.T @ y

# Sample from posterior
samples = np.random.multivariate_normal(mu, Sigma, 50)

# Plot
x_vals = np.linspace(0, 4, 100)

plt.scatter([1,2,3], y)

for a, b in samples:
    plt.plot(x_vals, a*x_vals + b, alpha=0.2)

plt.show()