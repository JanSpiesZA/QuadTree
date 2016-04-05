int GridWidth = 100;
int GridHeight = 100;

int screenWidth = 640;
int screenHeight = 640;
int tileSize = 40;

int level = 4;

int maxHistogramX = int(screenWidth/tileSize);
int maxHistogramY = int(screenHeight/tileSize);

color allNodesColor = color(255,255,230);
int allNodesStrokeWeight = 2;
color finalPathColor = color(0,255,0);
int finalPathStrokeWeight = 10;

boolean mix = false;
boolean runOnce = false;

PVector startPos = new PVector(tileSize/4, tileSize/2);      //Starting position of robot
PVector goalPos = new PVector(screenWidth-tileSize/4, screenHeight-tileSize/2);        //Goal position of robot


Tile tile[][] = new Tile[maxHistogramX][maxHistogramY];

//List to hold all the node information. Nodes will be used for path planning after quad tree creation

//ArrayList<Node> openList = new ArrayList<Node>();
ArrayList<Integer> closedList = new ArrayList<Integer>();
ArrayList<Node> allNodes = new ArrayList<Node>();
ArrayList<Integer> openList = new ArrayList<Integer>();
ArrayList<Integer> finalPath = new ArrayList<Integer>();

void setup()
{
  //surface.setResizable(true);
  //surface.setSize(screenWidth,screenHeight);
  
  size(640,640);
  
  //Sets up a 2D array which will hold the world Tiles
  for (int x = 0; x < maxHistogramX; x++)
  {
    for (int y = 0; y < maxHistogramY; y++)
    {
      tile[x][y] = new Tile();
    }
  }  
}

void draw()
{  
  //Draw executes once only to refresh screen thereafter screen is refreshed on mouse click
  if (!runOnce)
  {
    updateWorld();
    runOnce = true;
  }
}

//Recursively generates the quad tree nodes
void doQuadTree(int _topLeftX, int _topLeftY, int _sizeW, int _sizeH, int _level)
{ 
  //If no mix is found in a quad - draw a node
  if (!findMix(_topLeftX, _topLeftY, _sizeW, _sizeH))
  { 
   float nodeX = (_topLeftX + float(_sizeW)/2)*tileSize;  //cast _sizeW as float in order to do math
   float nodeY = (_topLeftY + float(_sizeH)/2)*tileSize;  //cast _sizeH as float in order to do math   
      
   allNodes.add(new Node(nodeX,nodeY,allNodes.size()));    //Add new node to allNodes arrayList
   
   //ellipse(nodeX,nodeY, 10,10);   //Draws an ellipse to indicate node x,y 
   //textSize(50);
   //fill(0);
   //text(allNodes.size()-1,nodeX-8,nodeY-8);
   return;
  }
  //If a mixed quad is found and it is the last level DO NOT draw a node, just return
  else if (findMix(_topLeftX, _topLeftY, _sizeW, _sizeH) && _level == 0)
  {
    return;
  }
  else
  {
    //If a mixed quad is found: Divide the quad into four new quads   
    //Top left quad    
    doQuadTree(_topLeftX, _topLeftY, _sizeW/2, _sizeH/2, _level-1);
    
    //Top right quad
    doQuadTree(_topLeftX + _sizeW/2, _topLeftY, _sizeW/2, _sizeH/2, _level-1);
    
    //Btm left quad    
    doQuadTree(_topLeftX, _topLeftY + _sizeH/2, _sizeW/2, _sizeH/2, _level-1);
    
    //Btm right quad    
    doQuadTree(_topLeftX + _sizeW/2, _topLeftY + _sizeH/2, _sizeW/2, _sizeH/2, _level-1);
  }  
}

//Function returns TRUE if an occupied tile is found within a certain square of tiles
boolean findMix(int _topLeftX, int _topLeftY, int _sizeW, int _sizeH)
{
  noFill();
  stroke (0);
  strokeWeight(5);
  rect (_topLeftX * tileSize, _topLeftY * tileSize, _sizeW * tileSize, _sizeH * tileSize);
  
  for (int x = 0; x < _sizeW; x++)
  {
    for (int y = 0; y < _sizeH; y++)
    {
      if (tile[x+_topLeftX][y+_topLeftY].gravity == 1)
      {        
        return true;
      }      
    }
  }
  return false;
}


//Function which draws all the tiles of the world
void drawWorld()
{
  for (int x = 0; x < maxHistogramX; x++)
  {
    for (int y = 0; y < maxHistogramY; y++)
    {
      stroke(0);
      strokeWeight(1);
      fill(tile[x][y].gravityCol);
      rect(x*tileSize,y*tileSize, tileSize,tileSize);
    }
  }
}


void mousePressed()
{
  if (mousePressed && (mouseButton == LEFT)) 
  {
    //Update a specific tile's gravity to indicate obstacle or not
    tile[int(mouseX/tileSize)][int(mouseY/tileSize)].gravity *= -1;    
    tile[int(mouseX/tileSize)][int(mouseY/tileSize)].update();    
    
    updateWorld();
    
  }
}



void nodeLink()
{
  boolean intersect = false;
  
  //Looks at each node to find out which other nodes it is connected to
  for (int i = 0; i < allNodes.size(); i++)    //
  {    
    Node n1 = allNodes.get(i);
    
    for (int j = 0; j < allNodes.size(); j++)
    {    
      Node n2 = allNodes.get(j);
        
      for (int k = 0; k < maxHistogramX; k++)
      {
        for (int l = 0; l < maxHistogramY; l++)
        {
          //Test for intersect between current box and line
          //  If the tile's has gravity and the line crosses the tile then Intersect flag is set
          if ((tile[k][l].gravity == 1) && (line_box_xywh(n1.nodeXPos,n1.nodeYPos,n2.nodeXPos,n2.nodeYPos,k*tileSize,l*tileSize,tileSize,tileSize)))
          {
            intersect = true;            
          }
        }
      }
      
      //Test to see if intersect between box and line happend
      //  If it did intersect then reset flag, if not then draw a line to indicate a path between nodes
      if (intersect)
      {        
        intersect = false;
      }
      else
      {        
        stroke(allNodesColor);   
        strokeWeight(allNodesStrokeWeight);
        line(n1.nodeXPos,n1.nodeYPos,n2.nodeXPos,n2.nodeYPos);        
        if(!(n1.nodeXPos == n2.nodeXPos && n1.nodeYPos == n2.nodeYPos))      //Does not link to itself
        {
          n1.nodeConnectedTo.add(j);
        }
       }     
    }
  }
}

void keyPressed()
{
  switch (key)
  {
    case 's':  
      startPos.x = mouseX;
      startPos.y = mouseY;      
      break;
    case 'g':
      goalPos.x = mouseX;
      goalPos.y = mouseY;      
      break;
  }
  updateWorld();
}

void calcH()
{
  for (int k = 0; k < allNodes.size(); k++)
  {
    Node n = allNodes.get(k);
    if (n.nodeType != "GOAL")
    {
      n.H = dist(n.nodeXPos, n.nodeYPos, goalPos.x, goalPos.y);       
    }
  }
}


void findPath()
{ 
  int currentNodeID = 0;
  int startNodeID = 0;
  boolean foundPath = false;
  
  //Clears both lists when starting a new path finding cycle
  openList.clear();
  closedList.clear();
  finalPath.clear();
  
  calcH();      //Function used to calculate the H value of all nodes
  
  //Using the following website: http://homepages.abdn.ac.uk/f.guerin/pages/teaching/CS1013/practicals/aStarTutorial.htm  
  //Step 1
  //Find the START node ID and add it to the openList
  for (int k = 0; k < allNodes.size(); k++)      //Go through the entire allNodes list and find the START node
  {
    
    if (allNodes.get(k).nodeType == "START")
    {
      openList.add(k);                
      currentNodeID = allNodes.get(k).nodeID;  
      startNodeID = currentNodeID;
    }
  }
  
  println("Current openList: "+openList);
  println ("Current closedList :"+closedList+"\n");
  
  //LOOP to work through the openList in order to test all possible routes
  while (openList.size() > 0)
  {    
    //Step 2:
    //Adds the nodes connected to current node onto the open list
    //Set nodes in connectedList parentIDs to currentNode
    //Update G values. G = parentNode.G + distance from parent node  
    
    //Find the lowest F score
    float smallestF = 99999.0;           
    
    for (int k=0; k < openList.size(); k++)
    {
      
      if (allNodes.get(openList.get(k)).F < smallestF)
      {
        smallestF = allNodes.get(openList.get(k)).F;
        currentNodeID = openList.get(k);
      }
    }
    
    println("Closest node is "+currentNodeID+" with F of "+smallestF);
    
    //Move node with lowest F score to closedList
    openList.remove(openList.indexOf(currentNodeID));
    closedList.add(currentNodeID);
    
    Node currentNode = allNodes.get(currentNodeID);            
    for (int k = 0; k < currentNode.nodeConnectedTo.size(); k++)
    { 
      Node openListNode = allNodes.get(currentNode.nodeConnectedTo.get(k));              
      //Test to see if node is in closedList. If it is then it means that, that node
      //    has already been explored and should be ignored
      if (closedList.indexOf(openListNode.nodeID) == -1)
      {
        //Test if nodeConnectedTo number is already in openList, if not then add
        if (openList.indexOf(openListNode.nodeID) == -1)
        {
          openList.add(openListNode.nodeID);      
          openListNode.parentID = currentNodeID;
          openListNode.G = currentNode.G + dist(openListNode.nodeXPos,openListNode.nodeYPos, currentNode.nodeXPos, currentNode.nodeYPos);
          openListNode.F = openListNode.G + openListNode.H;                
          println("Node "+openListNode.nodeID+"'s H: "+openListNode.H+" G: "+openListNode.G+" and F: "+openListNode.F);                
        }
        else
        {     
          println(openListNode.nodeID+" already in openList");
          float _dist = dist(currentNode.nodeXPos, currentNode.nodeYPos, openListNode.nodeXPos, openListNode.nodeYPos);
          println("Distance from "+currentNode.nodeID+" to "+openListNode.nodeID+" is "+_dist);
          println("Current node G: "+currentNode.G+" openListNode.G: "+openListNode.G);
          println((currentNode.G + _dist));
          if ((currentNode.G + _dist) < openListNode.G)
          {
            openListNode.G = currentNode.G + _dist;
            openListNode.parentID = currentNode.nodeID;
            println("Node "+openListNode.nodeID+" new G value = "+openListNode.G);
          }
        }
      }
    }
    
    println("Current openList: "+openList);
    println("Current closedList :"+closedList+"\n");
  }
  //LOOP EINDIG HIERSO
  
  //Create final path by starting at the GOAL node and adding parent nodes until the START node is reached
  
  int startPathID = -1;
  
  //Find the ID of the GOAL node
  for (int k = 0; k < allNodes.size(); k++)
  {
    if (allNodes.get(k).nodeType == "GOAL")
    {
      startPathID = allNodes.get(k).nodeID;
      finalPath.add(startPathID);
    }
  }
  
  while( startPathID != startNodeID)
  {
    if (startPathID == -1)
    {
      println ("No Path");
      foundPath = false;
      break;
    }
    else
    {
      startPathID = allNodes.get(startPathID).parentID;
      finalPath.add(startPathID); 
      foundPath = true;
    }
  }
  
  
  if (foundPath)
  {
    println("Shortest Path: "+finalPath);
    for (int k = 0; k < finalPath.size()-1; k++)
    {
      strokeWeight(finalPathStrokeWeight);
      stroke(finalPathColor);
      line(allNodes.get(finalPath.get(k)).nodeXPos, allNodes.get(finalPath.get(k)).nodeYPos,
            allNodes.get(finalPath.get(k+1)).nodeXPos, allNodes.get(finalPath.get(k+1)).nodeYPos);
    }
  }
}

void updateWorld()
{
  //clear();  
  drawWorld(); 
      
  allNodes.clear();
          
  allNodes.add( new Node(startPos.x, startPos.y, "START", allNodes.size()));
  allNodes.add( new Node(goalPos.x, goalPos.y, "GOAL", allNodes.size()));
  
  doQuadTree(0,0,maxHistogramX, maxHistogramY,level);
  println("\nNumber of allNodes: "+allNodes.size());        //Print the total number of nodes    
  nodeLink();
  
  findPath();
  
  for (Node n: allNodes)
  {
   n.display();     
  }
}