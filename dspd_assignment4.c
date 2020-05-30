#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#define INITSIZE 4
#define Max 20

typedef enum{false,true} bool;

typedef struct AdjacentNode_tag 
{ 
    int dest;
	int cost; 
    struct AdjacentNode_tag* next; 
}AdjacentNode;

typedef struct AdjListNode_tag
{
	int Node_number;
	AdjacentNode* Adjlist;
	bool adjacent_exist;	
}AdjListNode;

typedef struct Graph_tag 
{ 
	char nature;                      // if nature = 'd' it means directed and if 'u' then undirected
	char weighted;
    int no_of_vertices; 
    AdjListNode arr[Max]; 
    int occupied;
}Graph;

AdjacentNode* newAdjacentNode(int dest, int cost) 
{ 
    AdjacentNode* newNode = (AdjacentNode*) malloc(sizeof(AdjacentNode)); 
    newNode->dest = dest;
    newNode->cost = cost;
	newNode->next = NULL; 
    return newNode; 
}

Graph* createGraph(int V,char nature,char weighted) 
{ 
    Graph* graph = (Graph*) malloc(sizeof(Graph)); 
    graph->no_of_vertices = V;
	graph->nature = nature;
	graph->weighted = weighted;
	graph->occupied=0;
	int i;
	for(i=0;i<Max;i++)
	{
		graph->arr[i].adjacent_exist=false;
		graph->arr[i].Adjlist=NULL;
		graph->arr[i].Node_number=-1;	
	}  
    return graph; 
}  

void addNode(Graph* graph,int node_no)
{
    graph->arr[graph->occupied].Node_number=node_no;
    graph->occupied++;
}

void addingEdge(Graph* graph,int src,int dest,int cost)
{
	AdjacentNode *ptr,*lptr;
	if(graph->weighted=='n')
	{
		cost=1;
	}
	ptr = newAdjacentNode(dest,cost);
	lptr = graph->arr[src].Adjlist;
	if(lptr==NULL)
	{
		graph->arr[src].adjacent_exist = true;
		graph->arr[src].Adjlist=ptr;
		printf("Edge added from %d to %d \n",src,dest);
	}
	else
	{
		while(lptr->next!=NULL)
		{
			lptr=lptr->next;
		}
		lptr->next=ptr;
		printf("Edge added from %d to %d \n",src,dest);
	}
}

void addEdge(Graph* graph,int src,int dest,int cost)
{
	addingEdge(graph,src,dest,cost);
	if(graph->nature=='u')
	{
		addingEdge(graph,dest,src,cost);
	}
}

void deletingEdge(Graph* graph,int src,int dest)
{
	if(!(graph->arr[src].adjacent_exist))
	{
		printf("No such source vertex exists in the graph \n ");
	}
	else
	{
		AdjacentNode *ptr,*prev;
		ptr = graph->arr[src].Adjlist;
		prev=NULL;
		int f=0;
		while(ptr && f==0)
		{
			if(ptr->dest==dest)
			{
				f=1;
			}
			else
			{
				prev=ptr;
				ptr=ptr->next;
			}
		}
		
		if(f==1)
		{
			if(prev==NULL)
			{
				graph->arr[src].Adjlist = ptr->next;
				printf("Edge deleted from %d to %d \n",src,ptr->dest);
				free(ptr);
			}
			else
			{
				prev->next=ptr->next;
				printf("Edge deleted from %d to %d \n",src,ptr->dest);
				free(ptr);
			}
		}
	}
}

void deleteEdge(Graph* graph,int src,int dest)
{
	deletingEdge(graph,src,dest);
	if(graph->nature=='u')
	{
		deletingEdge(graph,dest,src);
	}
}

void deleteList(Graph *graph,int node_no)
{
	AdjacentNode* head,*ptr;
	head= graph->arr[node_no].Adjlist;
	while(head!=NULL)
	{
		ptr = head;
		head=ptr->next;
		printf("Edge deleted from %d to %d \n",node_no,ptr->dest);
		free(ptr);
	}
	graph->arr[node_no].Adjlist=NULL;
	graph->arr[node_no].adjacent_exist=false;
}

void deleteNode(Graph *graph,int node_no)
{
	if(graph->occupied <= node_no)
	{
		printf("No such node exists in the graph \n");
	}
	else
	{
		int j;
		if(!(graph->arr[node_no].adjacent_exist))
		{
			graph->arr[node_no].Node_number=-1;
		}
		else
		{
			deleteList(graph,node_no);
			graph->arr[node_no].Node_number=-1;
		}
		graph->no_of_vertices--;
		for(j=0;j<graph->occupied;j++)
		{
			if(j!=node_no)
			{
				deletingEdge(graph,j,node_no);
			}
		}
	}
}

typedef struct Queue_tag 
{ 
    int front, rear, size; 
    int capacity; 
    int* array; 
}Queue; 

Queue* createQueue(int capacity) 
{ 
    Queue* queue = (Queue*) malloc(sizeof(Queue)); 
    queue->capacity = capacity; 
    queue->front = queue->size = 0;  
    queue->rear= capacity-1; 
    queue->array = (int*) malloc(queue->capacity * sizeof(int)); 
    return queue; 
} 

bool isEmpty(Queue* queue) 
{  
	if(queue->size == 0)
	return true;
	else
	return false;
} 

void enqueue(Queue* queue, int item) 
{ 
    if (queue->size < queue->capacity) 
    {
	  
    	queue->rear = (queue->rear + 1)%queue->capacity; 
    	queue->array[queue->rear] = item;
    	queue->size = queue->size + 1;  
	}
} 
  
int dequeue(Queue* queue) 
{ 
    int item;
    if (!isEmpty(queue))
	{
		item = queue->array[queue->front]; 
		queue->front = (queue->front + 1)%queue->capacity; 
    	queue->size = queue->size - 1; 
	}
	else
	{
		item = -1;
	}
	return item;
} 

void BFT(Graph* g)         // Breadth First Traversal
{
	int i,item,d;
	bool visited[Max];
	for(i=0;i<Max;i++)
	{
		visited[i] = false;
	}
	Queue* q;
	q = createQueue(Max);
	AdjacentNode* ptr;
	printf("\n Breadth First Traversal of Graph is -:  ");
	for(i=0;i<g->occupied;i++)    // this loop is used if the graph is disconnected
	{
		if(g->arr[i].Node_number!=-1 && visited[i]==false)
		{
			
			visited[i]=true;
			enqueue(q,i);
			while(!isEmpty(q))
			{
				item = dequeue(q);
				printf("%d ",item);
				ptr = g->arr[item].Adjlist;
				while(ptr)
				{
					d=ptr->dest;
					if(visited[d]==false)
					{
						visited[d]=true;
						enqueue(q,d);	
					}
					ptr=ptr->next;			
				}	
			}
		}
	}
	printf("\n");
	
}

bool BFS(Graph* g,int data)        // Breadth First Search
{
	int item,d,i;
	bool visited[Max];
	for(i=0;i<Max;i++)
	{
		visited[i] = false;
	}
	Queue* q;
	q = createQueue(Max);
	AdjacentNode* ptr;
	for(i=0;i<g->occupied;i++)    // this loop is used if the graph is disconnected
	{
		if(g->arr[i].Node_number!=-1 && visited[i]==false)
		{
			visited[i]=true;
			enqueue(q,i);
			while(!isEmpty(q))
			{
				item = dequeue(q);
				if(item==data)
				{
					return true;
				}
				ptr = g->arr[item].Adjlist;
				while(ptr)
				{
					d=ptr->dest;
					if(visited[d]==false)
					{
						visited[d]=true;
						enqueue(q,d);	
					}
					ptr=ptr->next;			
				}	
			}
		}
	}
	return false;
}



Graph* getTranspose(Graph* graph) 
{ 
    Graph* g=createGraph(graph->no_of_vertices,'d',graph->weighted);
	int v,d;
	for(v=0;v<g->no_of_vertices;v++)
	{
		addNode(g,v);
	}
	AdjacentNode* ptr; 
    for (v = 0; v < g->occupied; v++) 
    {   
        ptr = graph->arr[v].Adjlist;
	    while(ptr)
		{
			d=ptr->dest;
			addEdge(g,d,v,ptr->cost);
            ptr=ptr->next;	   	
		}  
    } 
    return g; 
} 

void DFT_util(Graph* g,bool visited[],int v)
{
	int d;
	AdjacentNode* ptr;
	visited[v]=true;
	printf("%d ",g->arr[v].Node_number);
	ptr=g->arr[v].Adjlist;
	while(ptr)
	{
		d=ptr->dest;
		if(!visited[d])
		{
			DFT_util(g,visited,d);
		}
		ptr=ptr->next;
	}
}

void DFT(Graph* g,bool* connect)
{
	int i;
	bool visited[Max];
	bool search=false;
	*connect = true;
	for(i=0;i<Max;i++)
	{
		visited[i] = false;
	}
	printf("\n Depth First Traversal of Graph is -: ");
	for(i=0;i<g->occupied;i++)
	{
		if(g->arr[i].Node_number!=-1 && !visited[i])
		{
			if(i>0)
			{
				*connect=false;
			}
			DFT_util(g,visited,i);
		}
	}
	printf("\n");
	if(g->nature=='d' && *connect==true)                      //  Kosaraju’s DFS algorithm is used here  
	{
		printf("\n DFT for Transpose Graph to determine connectivity \n");
		Graph* gt = getTranspose(g);
		for(i=0;i<gt->occupied;i++)
		{
			visited[i]=false;
		}
		for(i=0;i<gt->occupied;i++)
		{
			if(gt->arr[i].Node_number!=-1 && !visited[i])
			{
				if(i>0)
				{
					*connect=false;
				}
				DFT_util(gt,visited,i);
			}	
		} 
	}
	printf("\n");
}


void DFS_Search(Graph* g,bool visited[],int v,bool* search,int data)
{
	int d,f=0;
	AdjacentNode* ptr;
	visited[v]=true;
	ptr=g->arr[v].Adjlist;
	while(ptr && f==0)
	{
		d=ptr->dest;
		if(!visited[d])
		{
			if(d==data)
			{
				*search=true;
				f=1;
			}
			else
			{
				DFS_Search(g,visited,d,search,data);
			}
		}
		ptr=ptr->next;
	}
}


bool DFS(Graph* g,int data)
{
	int i,f=0;
	bool visited[Max];
	bool retval=false;
	for(i=0;i<Max;i++)
	{
		visited[i] = false;
	}
	for(i=0;i<g->occupied && f==0;i++)
	{
		if(g->arr[i].Node_number!=-1 && !visited[i])
		{
			if(i==data)
			{
				retval = true;
			}
			else
			{
				DFS_Search(g,visited,i,&retval,data);
			}
		}
	}
	return retval;	
}

typedef struct Stack_tag
{ 
    int top; 
    int capacity; 
    int* array; 
}Stack; 
 
Stack* createStack(int capacity) 
{ 
    Stack* stack = (Stack*)malloc(sizeof(Stack)); 
    stack->capacity = capacity; 
    stack->top = -1; 
    stack->array = (int*)malloc(stack->capacity * sizeof(int)); 
    return stack; 
}  

bool Empty(Stack* stack) 
{ 
    if(stack->top==-1)
    {
    	return true;
	}
	else
	{
		return false;
	}
} 

void push(Stack* stack, int item) 
{ 
    if (stack->top==stack->capacity-1)
	{ 
    	return;
	}
    stack->array[++stack->top] = item;  
} 
   
int pop(Stack* stack) 
{ 
    if (!Empty(stack))
	{  
    	return stack->array[stack->top--];
    	
	}
}  

void topological_util(Stack* s,Graph* g,bool visited[],int v)
{
	int d;
	visited[v]=true;
	AdjacentNode* ptr;
	ptr = g->arr[v].Adjlist;
	while(ptr)
	{
		d=ptr->dest;
		if(!visited[d])
		{
			topological_util(s,g,visited,d);
		}
		ptr=ptr->next;
	}
	push(s,g->arr[v].Node_number);
}

void topological_sort(Graph* g)
{
	int i,d;
	Stack* s = createStack(Max);
	bool visited[Max];
	for(i=0;i<Max;i++)
	{
		visited[i]=false;
	}
	for(i=0;i<g->occupied;i++)
	{
		if(g->arr[i].Node_number!=-1 && !visited[i])
		{
			topological_util(s,g,visited,i);
		}
	}
	printf("\n Topological sort of graph is :- ");
	while(!Empty(s))
	{
		d = pop(s);
		printf("%d ",d);
	}
	printf("\n");
	
}


bool detectHelper_directed(Graph* g,bool visited[],bool recstack[],int v)
{
	bool retval;
	retval = false;
	if(!visited[v])
	{
		int d,f;
		f=0;
		visited[v]=true;
		recstack[v]=true;
		AdjacentNode* ptr;
		ptr = g->arr[v].Adjlist;
		while(ptr && f==0)
		{
			d=ptr->dest;
			if(recstack[d])
			{
				retval = true;
				f=1;
			}
			else
			{
				retval = detectHelper_directed(g,visited,recstack,d);
				if(retval==true)
				{
					f=1;
				}
			}
			
			ptr = ptr->next;
		}
    }
    recstack[v]=false;
    return retval;
}

bool DetectGraph_directed(Graph* g)
{
	bool retval;
	retval = false;
	int i,f;
	f=0;
	bool visited[Max];
	bool recstack[Max];
	for(i=0;i<Max;i++)
	{
		visited[i] = false;
		recstack[i] = false;
	}
	for(i=0;i<g->no_of_vertices && f==0;i++)
	{
		if(g->arr[i].Node_number!=-1)
		{
			if(detectHelper_directed(g,visited,recstack,i))
			{
				retval = true;
				f=1;
			}
		}	
	}
	return retval;
}

bool detectHelper_undirected(Graph* g,bool visited[],int v,int parent)
{
	int d;
	visited[v]=true;
	AdjacentNode* ptr;
	ptr = g->arr[v].Adjlist;
	while(ptr)
	{
		d=ptr->dest;
		if(!visited[d])
		{
			if(detectHelper_undirected(g,visited,d,v))
			{
				return true;
			}
		}
		else if(d != parent)
		{
			return true;
		}
		ptr=ptr->next;
	}
	return false;
}

bool DetectGraph_undirected(Graph* g)
{
	int i;
	bool visited[Max];
	for(i=0;i<Max;i++)
	{
		visited[i]=false;
	}
	for(i=0;i<g->occupied;i++)
	{
		if(g->arr[i].Node_number!=-1 && !visited[i])
		{
			if(detectHelper_undirected(g,visited,i,-1))
			{
				return true;
			}
		}
	}
	return false;
}

int min_dist(int dist[],bool found[],int n)
{
	int i,min=999999;
	int p;
	for(i=0;i<n;i++)
	{
		if(!found[i])
		{
			if(dist[i]<=min)
			{
				min=dist[i];
				p=i;
			}
		}
	}
	return p;
}

bool isDone(bool found[],int n)
{
	int i,f=0;
	for(i=0;i<n && f==0;i++)
	{
		if(found[i]==false)
		{
			f=1;
		}
	}
	if(f==1)
	{
		return false;
	}
	return true;
}

void Dijaktra_shortest_path(Graph* g,int src)          // Dijaktra Algorithm
{
	int i,d,p,f,j;
	bool found[g->occupied];
	int dist[g->occupied];
	int parent[g->occupied];
	for(i=0;i<g->occupied;i++)
	{
		found[i]=false;
		dist[i]=999999;
		parent[i]=-1;
	}
	found[src]=true;
	dist[src]=0;
	AdjacentNode* ptr;
	ptr = g->arr[src].Adjlist;
	while(ptr)
	{
		d=ptr->dest;
		dist[d]=ptr->cost;
		parent[d]=src;
		ptr=ptr->next;
	}
	while(!isDone(found,g->occupied))
	{
		p = min_dist(dist,found,g->occupied);
		found[p]=true;
		ptr=g->arr[p].Adjlist;
		while(ptr)
		{
			d=ptr->dest;
			if((ptr->cost + dist[p])<dist[d])
			{
				dist[d]=ptr->cost+dist[p];
				parent[d]=p;
			}
			ptr=ptr->next;
		}
	}
	Stack* s=createStack(Max);
	for(i=0;i<g->occupied;i++)
	{
		if(i!=src)
		{
			if(dist[i]>=999999)
			{
				printf("\nShortest path cost from %d to %d is:  INF\n",src,i);
			}
			else
			{
				printf("\nShortest path cost from %d to %d is:  %d \n",src,i,dist[i]);
			}
			printf("Shortest Path from %d to %d is ",src,i);
			f=0;
			j=i;
			while(f==0)
			{
				push(s,j);
				d=parent[j];
				if(d==src)
				{
					f=1;
				}
				j=d;
			}
			push(s,j);
			while(!Empty(s))
			{
				d = pop(s);
				if(d!=-1)
				{
					printf("-> %d ",d);
				}
			}
			printf("\n");
		}
	}
}




int Cost(Graph* g,int u,int v)
{
    if(u==v)
    {
        return 0;
    }
    int cost=999999;
    AdjacentNode* ptr=g->arr[u].Adjlist;
    AdjacentNode* temp=ptr;
    while(temp!=NULL)
    {
        if(temp->dest==v)
        {
            cost=temp->cost;
        }
        temp=temp->next;
    }
    return cost;
}

void printCosts(int Dist[],int sz,int num) //Function only works with index==node_number
{
    printf("\nCosts from vertex %d\n",num);
    int i;
    for(i=0;i<sz;i++)
    {
    	if(Dist[i]==999999)
    	{
    		printf("Vertex %d\tINF\n",i);
		}
		else
		{
        	printf("Vertex %d\t%d\n",i,Dist[i]);
    	}
	}
}

void printPaths(int Path[],int sz,int num)//Function only works with index==node_number
{
    printf("\n\nPaths from vertex %d\n",num);
    int i;
    for(i=0;i<sz;i++)
    {
        printf("\nShortest Path from %d to %d\t",num,i);
        if(Path[i]==num)
        {
            if(num==i)
            {
                printf("itself\n");
            }
            else
            {
              printf("%d--->%d\n",num,i);
            }
                   
        }
        else
        {
            Stack* s;
            printf("%d--->",num);
            s=createStack(Max);
            int parent=Path[i];
            do
            {
                push(s,parent);
                parent=Path[parent];
            }while(parent!=num);
            
            while(!Empty(s))
            {
            	printf("%d-->",pop(s));
			}
            printf("%d\n",i);
        }
        
    }
}

void APSP(Graph* graph)
{
    int Dist[graph->occupied][graph->occupied];
    int Path[graph->occupied][graph->occupied];
    int i,j,k;
	int N=graph->occupied;
    for(i=0;i<N;i++)
    {
        for(j=0;j<N;j++)
        {
            Dist[i][j]=Cost(graph,graph->arr[i].Node_number,graph->arr[j].Node_number); //Making A^-1
            Path[i][j]=i;
        }
    }

    for(k=0;k<N;k++)
    {
        for(i=0;i<N;i++)
        {
            for(j=0;j<N;j++)
            {
                if(Dist[i][j]>Dist[i][k] + Dist[k][j])
                {
                    Dist[i][j]=Dist[i][k] + Dist[k][j];
                    Path[i][j]=graph->arr[k].Node_number;
                }
            }
        }
    }
    for(i=0;i<N;i++) //Extracting info for each vertex and printing
    //info for each vertex being one row in the dist and path matrices
    {
        int temp[N];int temp2[N],j;
        for(j=0;j<N;j++)
        {
            temp[j]=Dist[i][j];
            temp2[j]=Path[i][j];
        }
        printCosts(temp,N,i);
        printPaths(temp2,N,i);
    }
}

void PrintMST(int key[],int parent[],int n)
{
	int i;
	printf("Edge \t Weight \n");
	for(i=1;i<n;i++)
	{
		printf("%d--->%d    %d \n",parent[i],i,key[i]);
	}
}

void MST(Graph* g)
{
	int i,minkey,j,count;
	int parent[g->occupied],key[g->occupied];
	bool keyset[g->occupied];
	for(i=0;i<g->occupied;i++)
	{
		parent[i]=-1;
		key[i]=INT_MAX;
		keyset[i]=false;
	}
	key[0]=0;
	AdjacentNode*ptr;
	for(count=0;count<g->no_of_vertices-1;count++)
	{
		minkey = min_dist(key,keyset,g->occupied);
		keyset[minkey]=true;
		for(j=0;j<g->no_of_vertices;j++)
		{
			if(Cost(g,minkey,j) && keyset[j]==false && Cost(g,minkey,j)<key[j])
			{
				key[j]=Cost(g,minkey,j);
				parent[j]=minkey;	
			}	
		}
	}
	int total_weight=0;
	for(i=0;i<g->occupied;i++)
	{
		total_weight += key[i];
	}
	printf("\n The total weight of MST is: %d \n",total_weight);
	PrintMST(key,parent,g->no_of_vertices);
}


void printAllPathsUtil(Graph* g,int s,int d,bool visited[],bool path[],int* path_tracker)
{
	int b=*path_tracker;
	visited[s]=true;
	path[b]=s;
	b++;
	
	if(s==d)
	{
		int i;
		printf("\n");
		for(i=0;i<b;i++)
		{
			printf("-->%d",path[i]);
		}
	}
	else
	{
		int a;
		AdjacentNode* ptr;
		ptr=g->arr[s].Adjlist;
		while(ptr!=NULL)
		{
			a=ptr->dest;
			if(!visited[a])
			{
				*path_tracker=b;
				printAllPathsUtil(g,a,d,visited,path,path_tracker);
			}
			ptr=ptr->next;
		}
	}
	b--;
	*path_tracker=b;
	visited[s]=false;
}

void Print_All_path(Graph* g)
{
	bool visited[g->occupied];
	int path[g->occupied],path_tracker=0,i;
	for(i=0;i<g->occupied;i++)
	{
		visited[i]=false;
		path[i]=-1;
	}
	int u,v;
	for(u=0;u<g->occupied;u++)
	{
		for(v=0;v<g->occupied;v++)
		{
			if(u!=v)
			{
				printf("\nTotal Number of Paths from %d to %d are as follows: \n",u,v);
				printAllPathsUtil(g, u, v, visited, path, &path_tracker); 
			}
		}
	}
}


void printGraph(Graph* graph) 
{ 
    int v; 
    for (v = 0; v < graph->no_of_vertices; v++) 
    { 
        AdjacentNode* pCrawl = graph->arr[v].Adjlist; 
        printf("\n Adjacency list of vertex %d\n head ", v); 
        while (pCrawl) 
        { 
            printf("-> %d", pCrawl->dest); 
            pCrawl = pCrawl->next; 
        } 
        printf("\n \n"); 
    }
	 
} 

int main()
{
	int i,a,V = 9; 
	bool connect;
    Graph* graph = createGraph(V,'u','y');
    for(i=0;i<V;i++)
    {
    	addNode(graph,i);
	}
	/*addEdge(graph, 0, 1,1); 
    addEdge(graph, 0, 4,1); 
    addEdge(graph, 1, 2,1); 
    addEdge(graph, 1, 3,1); 
    addEdge(graph, 1, 4,1); 
    addEdge(graph, 2, 3,1); 
    addEdge(graph, 3, 4,1); */
    addEdge(graph, 0, 1, 4); 
    addEdge(graph, 0, 7, 8); 
    addEdge(graph, 1, 2, 8); 
    addEdge(graph, 1, 7, 11); 
    addEdge(graph, 2, 3, 7); 
    addEdge(graph, 2, 8, 2); 
    addEdge(graph, 2, 5, 4); 
    addEdge(graph, 3, 4, 9); 
    addEdge(graph, 3, 5, 14); 
    addEdge(graph, 4, 5, 10); 
    addEdge(graph, 5, 6, 2); 
    addEdge(graph, 6, 7, 1); 
    addEdge(graph, 6, 8, 6); 
    addEdge(graph, 7, 8, 7); 
	printGraph(graph);
	BFT(graph);
	DFT(graph,&connect);
	if(graph->nature=='d')
	{
		if(DetectGraph_directed(graph))
		{
			printf("\n Graph contains a cycle \n");
			printf("\n Topological sort is not possible for this Graph \n");
		}
		else
		{
			printf(" \n Graph doesn't contain a cycle \n");
			topological_sort(graph);
		}
	}
	else
	{
		if(DetectGraph_undirected(graph))
		{
			printf("\n Graph contains a cycle \n");
		}
		else
		{
			printf("\n Graph doesn't conatain a cycle \n");
		}
	}
	
	if(connect==true)
	{
		printf("\n Graph is connected \n");	
	}
	else
	{
		printf("\n Graph is not connected \n");
	}
	Dijaktra_shortest_path(graph,0);
	APSP(graph);
	printf("\nEnter the Node no. you want to search in the graph \n");
	scanf("%d",&a);
	if(BFS(graph,a))
	{
		printf("\nNode_No. %d exists in the graph \n",a);	
	}
	else
	{
		printf("\nNode_No. %d doesn't exists in the graph \n",a);	
	}
	printf("\nEnter the Node no. you want to search in the graph \n");
	scanf("%d",&a);
	if(DFS(graph,a))
	{
		printf("\nNode_No. %d exists in the graph \n",a);	
	}
	else
	{
		printf("\nNode_No. %d doesn't exists in the graph \n",a);	
	}
	MST(graph);	
	Print_All_path(graph);		
	return 0;
}
