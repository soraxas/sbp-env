from unittest import TestCase

import numpy as np

from planners.rrdtPlanner import Node as NodeWithEdge
from utils.common import MagicDict, BFS, Tree, Node


class TestNode(TestCase):
    def test_index_op(self):
        n = Node(np.array([5.2, 2.5]))
        self.assertAlmostEqual(n[0], 5.2)
        self.assertAlmostEqual(n[1], 2.5)

    def test_equality(self):
        n = Node(np.array([5.2, 2.5]))
        self.assertEqual(n, Node(np.array([5.2, 2.5])))
        n = Node(np.array([2.4124, 53.623, 23.423]))
        self.assertEqual(n, Node(np.array([2.4124, 53.623, 23.423])))


class TestTree(TestCase):
    def setUp(self) -> None:
        self.tree = Tree(dimension=2)
        pos = np.array([1, 2])
        self.a = Node(pos)
        self.tree.add_vertex(self.a, pos)
        pos = np.array([4, 2])
        self.b = Node(pos)
        self.tree.add_vertex(self.b, pos)
        pos = np.array([4, 8])
        self.c = Node(pos)
        self.tree.add_vertex(self.c, pos)

    def test_nearby(self):
        nearby_nodes = list(self.tree.nearby(np.array([3, 4]), 2))
        self.assertTrue(any(np.isclose(self.a.pos, n.pos).all() for n in nearby_nodes))
        self.assertTrue(any(np.isclose(self.b.pos, n.pos).all() for n in nearby_nodes))

    def test_get_nearest(self):
        self.assertTrue(
            np.isclose(
                self.a.pos, self.tree.get_nearest(np.array([1.5, 1.5])).pos
            ).all()
        )
        self.assertTrue(
            np.isclose(self.b.pos, self.tree.get_nearest(np.array([4, 3])).pos).all()
        )
        self.assertTrue(
            np.isclose(self.c.pos, self.tree.get_nearest(np.array([4, 7])).pos).all()
        )


class TestMagicDict(TestCase):
    def setUp(self) -> None:
        self.m_dict = MagicDict()

    def test_add_attr(self):
        self.m_dict["foo"] = "bar"
        self.assertEqual(self.m_dict.foo, "bar")
        self.m_dict["num"] = 42
        self.assertEqual(self.m_dict.num, 42)

    def test_dict_remove_key(self):
        self.m_dict["foo"] = "bar"
        self.assertEqual(self.m_dict.foo, "bar")
        del self.m_dict["foo"]
        # should raise key error as attribute has been removed
        with self.assertRaises(KeyError):
            self.m_dict.foo

    def test_deepcopy_dict(self):
        self.m_dict["foo"] = "bar"
        self.assertEqual(self.m_dict.foo, "bar")
        import copy

        copied_dict = copy.deepcopy(self.m_dict)
        # alter value in the original dict
        self.m_dict["foo"] += "my second bar"
        # assert that the deep copied version has not altered variable
        self.assertEqual(copied_dict["foo"], "bar")


class NodeForTest(NodeWithEdge):
    def __init__(self, name: str):
        super().__init__(np.array([42]))
        self.name = name

    def __repr__(self):
        return f"Node<{self.name}>"


class TestBFS(TestCase):
    @staticmethod
    def _connects_node(parent: NodeWithEdge, child: NodeWithEdge):
        parent.edges.append(child)
        child.edges.append(parent)

    def setUp(self) -> None:
        """
        The graph structure looks like:

                          n0
                        / | \
                   n1_0 n1_1  n1_2
                  /  |         \
             n1_0_0  n1_0_1   n1_2_0
                      |
                    n1_0_1_0

        """
        self.n0 = NodeForTest("n0")
        self.n1_0 = NodeForTest("n1_0")
        self.n1_1 = NodeForTest("n1_1")
        self.n1_2 = NodeForTest("n1_2")
        self.n1_0_0 = NodeForTest("n1_0_0")
        self.n1_0_1 = NodeForTest("n1_0_1")
        self.n1_2_0 = NodeForTest("n1_2_0")
        self.n1_0_1_0 = NodeForTest("n1_0_1_0")
        self._connects_node(self.n0, self.n1_0)
        self._connects_node(self.n0, self.n1_1)
        self._connects_node(self.n0, self.n1_2)
        self._connects_node(self.n1_0, self.n1_0_0)
        self._connects_node(self.n1_0, self.n1_0_1)
        self._connects_node(self.n1_2, self.n1_2_0)
        self._connects_node(self.n1_0_1, self.n1_0_1_0)
        self.nodes_set = {
            self.n0,
            self.n1_0,
            self.n1_1,
            self.n1_2,
            self.n1_0_0,
            self.n1_0_1,
            self.n1_2_0,
            self.n1_0_1_0,
        }

    def test_has_no_next(self):
        bfs = BFS(self.n0, set())
        self.assertFalse(bfs.has_next(), "should contains no next node")

    def test_has_two_next(self):
        bfs = BFS(self.n1_0, {self.n1_0, self.n1_0_1, self.n1_0_1_0})
        self.assertTrue(bfs.has_next(), "should contains exactly two descendants")
        self.assertEqual(bfs.next().name, "n1_0")
        self.assertEqual(bfs.next().name, "n1_0_1")
        self.assertEqual(bfs.next().name, "n1_0_1_0")
        self.assertFalse(bfs.has_next(), "should contains exactly two descendants")

    def test_correct_returning_order_1(self):
        """
        BFS sequence should be [n0, n1_0, n1_1, n1_2,
        n1_0_0, n1_0_1, n1_2_0, n1_0_1_0]
        """
        bfs = BFS(self.n0, self.nodes_set)
        self.assertEqual(bfs.next().name, "n0")
        self.assertEqual(bfs.next().name, "n1_0")
        self.assertEqual(bfs.next().name, "n1_1")
        self.assertEqual(bfs.next().name, "n1_2")
        self.assertEqual(bfs.next().name, "n1_0_0")
        self.assertEqual(bfs.next().name, "n1_0_1")
        self.assertEqual(bfs.next().name, "n1_2_0")
        self.assertEqual(bfs.next().name, "n1_0_1_0")
        self.assertFalse(bfs.has_next())

    def test_correct_returning_order_2(self):
        """
        BFS sequence should be [n1_0_0, n1_0, n1_0_1, n0,
        n1_0_1_0, n1_1, n1_2, n1_2_0]
        """
        bfs = BFS(self.n1_0_0, self.nodes_set)
        self.assertEqual(bfs.next().name, "n1_0_0")
        self.assertEqual(bfs.next().name, "n1_0")
        self.assertEqual(bfs.next().name, "n0")
        self.assertEqual(bfs.next().name, "n1_0_1")
        self.assertEqual(bfs.next().name, "n1_1")
        self.assertEqual(bfs.next().name, "n1_2")
        self.assertEqual(bfs.next().name, "n1_0_1_0")
        self.assertEqual(bfs.next().name, "n1_2_0")
        self.assertFalse(bfs.has_next())
