    //Node包含路径代价，因为openset每次从中取fCost中最小值得点，openset采用堆的结构，每次加入点后自动堆排序
    public int gCost;
    public int hCost;
    public int fCost
    {
        get
        {
            return gCost + hCost;
        }
    }
    
    -----------------------------------------------
    //比较fCost,C#中的基本类型都提供了默认的比较算法，C#可以调用比较算法为基本类型的数组进行排序。
	若希望对自建类进行比较或排序，那么可以使用IComparable<T>和IComparer<T>接口。openSet需要用到堆类型数据
	public interface IHeapItem<T> : IComparable<T>
	{
	    int HeapIndex
	    {
	        get; set;
	    }
	}
	//自定义比较函数，
	public int CompareTo(Node nodeToCompare)
    {
        int compare = fCost.CompareTo(nodeToCompare.fCost);
        if (compare == 0)
        {
            compare = hCost.CompareTo(nodeToCompare.hCost);
        }
        return -compare;
    }
    //二叉堆排序主要功能
    private void _SortUp(T item)
    {
        int parentIndex = (item.HeapIndex - 1) / 2;

        while (true)
        {
            T parentItem = _items[parentIndex];

            if (item.CompareTo(parentItem) > 0)
                _Swap(item, parentItem);
            else
                break;

            parentIndex = (item.HeapIndex - 1) / 2;
        }
    }

    private void _SortDown(T item)
    {
        while (true)
        {
            int childLeftIndex = (item.HeapIndex * 2) + 1;
            int childRightIndex = (item.HeapIndex * 2) + 2;
            int swapIndex = 0;

            if (childLeftIndex < _currentItemCount)
            {
                swapIndex = childLeftIndex;

                if (childRightIndex < _currentItemCount)
                    if (_items[childLeftIndex].CompareTo(_items[childRightIndex]) < 0)
                        swapIndex = childRightIndex;

                if (item.CompareTo(_items[swapIndex]) < 0)
                    _Swap(item, _items[swapIndex]);
                else
                    return;
            }
            else
                return;
        }
    }

	------------------------------------
	//获取最短路径
	openSet.Add(_startNode);
    openSetContainer.Add(_startNode);
    while (openSet.Count > 0)
    ｛
	    currentNode = openSet.RemoveFirst();
	    openSetContainer.Remove(_startNode);
		............
	}
    //先将起点加入openSet，再取出fCost最小值点，即堆第一个值。
    //获取此点的邻居，
    //起点则parent点为null，遍历邻居非障碍点加入。
    if (parentNode == null)
        {
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    if (x == 0 && y == 0)
                        continue;

                    if (IsWalkable(x + currentNode.x, y + currentNode.y))
                    {
                        neighbours.Add(_grid[x + currentNode.x, y + currentNode.y]);
                    }
                }
            }
        }
       //非起点邻居点判断
        int xDirection = Mathf.Clamp(currentNode.x - parentNode.x, -1, 1);
        int yDirection = Mathf.Clamp(currentNode.y - parentNode.y, -1, 1);
		//判断是否水平方向
        if (xDirection != 0 && yDirection != 0)
        {
            //assumes positive direction for variable naming
            bool neighbourUp = IsWalkable(currentNode.x, currentNode.y + yDirection);
            bool neighbourRight = IsWalkable(currentNode.x + xDirection, currentNode.y);
            bool neighbourLeft = IsWalkable(currentNode.x - xDirection, currentNode.y);
            bool neighbourDown = IsWalkable(currentNode.x, currentNode.y - yDirection);
			//	当前方向上可走点判断，
            if (neighbourUp)
                neighbours.Add(_grid[currentNode.x, currentNode.y + yDirection]);

            if (neighbourRight)
                neighbours.Add(_grid[currentNode.x + xDirection, currentNode.y]);

            if (neighbourUp || neighbourRight)
                if (IsWalkable(currentNode.x + xDirection, currentNode.y + yDirection))
                    neighbours.Add(_grid[currentNode.x + xDirection, currentNode.y + yDirection]);
			//是否有强迫邻居
            if (!neighbourLeft && neighbourUp)
                if (IsWalkable(currentNode.x - xDirection, currentNode.y + yDirection))
                    neighbours.Add(_grid[currentNode.x - xDirection, currentNode.y + yDirection]);

            if (!neighbourDown && neighbourRight)
                if (IsWalkable(currentNode.x + xDirection, currentNode.y - yDirection))
                    neighbours.Add(_grid[currentNode.x + xDirection, currentNode.y - yDirection]);
        }
        else
        {
        	//y水平方向
            if (xDirection == 0)
            {
                if (IsWalkable(currentNode.x, currentNode.y + yDirection))
                {
                    neighbours.Add(_grid[currentNode.x, currentNode.y + yDirection]);

                   
                    if (!IsWalkable(currentNode.x + 1, currentNode.y))
                        if (IsWalkable(currentNode.x + 1, currentNode.y + yDirection))
                            neighbours.Add(_grid[currentNode.x + 1, currentNode.y + yDirection]);

                    if (!IsWalkable(currentNode.x - 1, currentNode.y))
                        if (IsWalkable(currentNode.x - 1, currentNode.y + yDirection))
                            neighbours.Add(_grid[currentNode.x - 1, currentNode.y + yDirection]);
                }
            }
            else
            {
				//x水平方向
                if (IsWalkable(currentNode.x + xDirection, currentNode.y))
                {
                    neighbours.Add(_grid[currentNode.x + xDirection, currentNode.y]);
                    if (!IsWalkable(currentNode.x, currentNode.y + 1))
                        neighbours.Add(_grid[currentNode.x + xDirection, currentNode.y + 1]);
                    if (!IsWalkable(currentNode.x, currentNode.y - 1))
                        neighbours.Add(_grid[currentNode.x + xDirection, currentNode.y - 1]);
                }
            }
        }
        ------------------------------------------------------------------------------
        //根据可走邻居，判断是否满足跳点条件，假设可走到邻居，判断是否可以到达跳点。
        //如果是斜方向，有强迫邻居，直接返回。
        if ((!_grid.IsWalkable(currentNode.x - xDirection, currentNode.y) && _grid.IsWalkable(currentNode.x - xDirection, currentNode.y + yDirection)) ||
                (!_grid.IsWalkable(currentNode.x, currentNode.y - yDirection) && _grid.IsWalkable(currentNode.x + xDirection, currentNode.y - yDirection)))
            {
                return currentNode;
            }
        //递归判断，斜方向，水平垂直方向可走，没有强迫邻居，继续斜方向寻找跳点。有则返回当前点。
        
        Node nextHorizontalNode = _grid.GetNodeFromIndex(currentNode.x + xDirection, currentNode.y);
        Node nextVerticalNode = _grid.GetNodeFromIndex(currentNode.x, currentNode.y + yDirection);
        
		if (_Jump(nextHorizontalNode, currentNode, xDirection, 0) != null || _Jump(nextVerticalNode, currentNode, 0, yDirection) != null)
		{
		    if (!_forced)
		    {
		        UnityEngine.Debug.Log(currentNode);
		        Node temp = _grid.GetNodeFromIndex(currentNode.x + xDirection, currentNode.y + yDirection);
		        if (temp != null && _grid.showDebug)
		            UnityEngine.Debug.DrawLine(new Vector3(currentNode.x, 1, currentNode.y), new Vector3(temp.x, 1, temp.y), Color.green, Mathf.Infinity);
		        return _Jump(temp, currentNode, xDirection, yDirection);
		    }
		    else
		    {
		        return currentNode;
		    }
		}
        //如果水平方向移动，没有强迫邻居，继续查找下一个点
		
        if (xDirection != 0)
        {
            //
            if ((_grid.IsWalkable(currentNode.x + xDirection, currentNode.y + 1) && !_grid.IsWalkable(currentNode.x, currentNode.y + 1)) ||
                (_grid.IsWalkable(currentNode.x + xDirection, currentNode.y - 1) && !_grid.IsWalkable(currentNode.x, currentNode.y - 1)))
            {
                _forced = true;
                return currentNode;
            }
        }
        else
        {
            if ((_grid.IsWalkable(currentNode.x + 1, currentNode.y + yDirection) && !_grid.IsWalkable(currentNode.x + 1, currentNode.y)) ||
                (_grid.IsWalkable(currentNode.x - 1, currentNode.y + yDirection) && !_grid.IsWalkable(currentNode.x - 1, currentNode.y)))
            {
                _forced = true;
                return currentNode;
            }
        }
        
        Node nextNode = _grid.GetNodeFromIndex(currentNode.x + xDirection, currentNode.y + yDirection);
        if (nextNode!= null && _grid.showDebug)
            UnityEngine.Debug.DrawLine(new Vector3(currentNode.x, 1, currentNode.y),
             new Vector3(nextNode.x, 1, nextNode.y), Color.green, Mathf.Infinity);
        return _Jump(nextNode, currentNode, xDirection, yDirection);

		//设置返回跳点的cost值，并加入openset。并更新堆排序
		int newGCost = currentNode.gCost + _GetDistance(currentNode, node);
        if (newGCost < node.gCost || !openSetContainer.Contains(node))
        {
            node.gCost = newGCost;
            node.hCost = _GetDistance(node, _targetNode);
            node.parent = currentNode;

            if (!openSetContainer.Contains(node))
            {
                openSetContainer.Add(node);
                openSet.Add(node);
            }
            else
            {
                openSet.UpdateItem(node);
            }
        }
        
        ---------------------------------------------------------
        //判断距离，因为cpu计算*和+速度比较快，所以在计算gCost和hCost值通过*和+采取近似值计算，格子和边的类比长度为，
        10，14（边如果是1，斜边为1.41....），根据xy上不同距离，进行映射。
	    private int _GetDistance(Node a, Node b)
	    {
	        int distX = Mathf.Abs(a.x - b.x);
	        int distY = Mathf.Abs(a.y - b.y);
	
	        if (distX > distY)
	            return 14 * distY + 10 * (distX - distY);
	
	        return 14 * distX + 10 * (distY - distX);
	 	}

		//2018腾讯移动游戏技术评审标准与实践案例中提到的jps算法优化部分在本项目中未更新
		

