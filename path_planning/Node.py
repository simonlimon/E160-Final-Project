class Node:

    def __init__(self, x, y, id):
        self.x = x
        self.y = y
        self.id = id

    def __eq__(self, other):
        return str(self) == str(other)

    def __repr__(self):
        return '<Node %s at (%s, %s)>' % (self.id, int(self.x), int(self.y))

    def __str__(self):
        return str(self.id)

    def __hash__(self):
        return hash(str(self))
