# refreshed
class Polynomial:
    def __init__(self, x2, x1, x0):
        self.x2 = x2
        self.x1 = x1
        self.x0 = x0

    def derivative(self):
        x0 = self.x1
        x1 = 2 * self.x2
        x2 = 0
        return Polynomial(x2, x1, x0)

    def integration(self):
        x2 = 0.5 * self.x1
        x1 = self.x0
        x0 = 0
        return Polynomial(x2, x1, x0)

    def get_value(self, x: float):
        value = self.x0 + self.x1 * x + self.x2 * x * x
        return value

    def add_constant(self, x0):
        p = Polynomial(self.x2, self.x1, self.x0 + x0)
        return p
