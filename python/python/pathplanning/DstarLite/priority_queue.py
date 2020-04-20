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
        return item[1]  # return the right tuple: item = ((priority),(id))

    def top(self):
        """
        :return: index of the smallest node in the priority queue
        """
        return self.queue[0][1]

    def top_key(self):
        """
        :return: key/priority value of the smallest node in the priority queue
        """
        self.queue.sort()
        #heapq.heapify(self.queue)
        #print(self.queue)
        if len(self.queue) > 0:
            return self.queue[0][0]
            # print(new_queue)
            #return heapq.nsmallest(1, self.queue)[0][0]
        else:
            # print('empty queue!')
            return float('inf'), float('inf')

    def remove(self, id: (int,int)):
        """
        :param node:
        :return:
        """
        self.queue = [e for e in self.queue if e[1] != id]
        heapq.heapify(self.queue)

    def update(self, id: (int,int), priority: (float,float)):
        self.remove(id=id)
        self.insert(item=id, priority=priority)

    def is_in_queue(self, id):
        ids = [q[1] for q in self.queue]
        return id in ids

    def __iter__(self):
        """
        :return:
        """
        for key, node in self.queue:
            yield node
