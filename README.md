# Distance between bus stops


## Dijkstra Implementation : Memory Limit Exceeded
```java
class Solution {
    public int distanceBetweenBusStops(int[] distance, int start, int destination) {
        if(start == destination)
            return 0;
        int n = distance.length;
        int[] shortestDistances = new int[n];
        Arrays.fill(shortestDistances, Integer.MAX_VALUE);
        shortestDistances[start] = 0;
        PriorityQueue<int[]> minHeap = new PriorityQueue<>((v1, v2) -> v1[1] - v2[1]);
        minHeap.offer(new int[]{start, 0});
        // create adjacency matrix
        int[][] adjMatrix = new int[n][n];
        boolean[] visited = new boolean[n];
        for(int i = 0; i < n; i++) {
            Arrays.fill(adjMatrix[i], -1);
        }
        for(int i = 0; i < distance.length; i++) {
            adjMatrix[i][(i+1) % n] = distance[i];
            adjMatrix[(i+1) % n][i] = distance[i];
        }
        while(!minHeap.isEmpty()) {
            int[] busStop = minHeap.poll();
            if(busStop[0] == destination)
                return busStop[1];
            if(visited[busStop[0]])
                continue;
            visited[busStop[0]] = true;
            // check neighboring stops
            for(int v = 0; v < n; v++) {
                if(adjMatrix[busStop[0]][v] != -1) {
                    int dU = busStop[1];
                    int distanceToNeighbor = adjMatrix[busStop[0]][v];
                    if(dU + distanceToNeighbor < shortestDistances[v]) {
                        shortestDistances[v] = dU + distanceToNeighbor;
                        minHeap.offer(new int[]{v, shortestDistances[v]});
                    }
                }
            }
        }
        return -1;
    }
}
```
