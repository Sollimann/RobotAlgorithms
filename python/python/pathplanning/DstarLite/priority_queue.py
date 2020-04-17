import heapq


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

    def top_key(self):
        """
        :return:

        print(self.queue)
        smallest_priority_list = heapq.nsmallest(1, self.queue)
        print("top_key: ".format(smallest_priority_list))
        if len(smallest_priority_list):
            return float('inf'), float('inf')
        smallest_element = smallest_priority_list[0][0]
        return smallest_element
                """
        self.queue.sort()
        # print(queue)
        if len(self.queue) > 0:
            smallest = self.queue[0][:2]
            smaller = smallest[0]
            return smaller
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
