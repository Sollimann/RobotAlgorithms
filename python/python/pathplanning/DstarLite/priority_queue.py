import heapq
from utils import *


class PriorityQueue:
    def __init__(self):
        self.queue = []

    def empty(self):
        """
        :return:
        """
        return len(self.queue) == 0

    def insert(self, item, priority):
        """
        :param item:
        :param priority:
        :return:
        """
        heapq.heappush(self.queue, (priority, item))

    def pop_top(self):
        """
        :return:
        """
        item = heapq.heappop(self.queue)
        return item[1]

    def top_key(self, s_current):
        """
        :return:
        """
        self.queue.sort()
        if len(self.queue) > 0:
            return self.queue[0][0]
            #print(new_queue)
            #return heapq.nsmallest(1, self.queue)[0][0]
        else:
            # print('empty queue!')
            return float('inf'), float('inf')

    def delete(self, node):
        """
        :param node:
        :return:
        """
        self.queue = [e for e in self.queue if e[1] != node]

    def __iter__(self):
        """
        :return:
        """
        for key, node in self.queue:
            yield node
