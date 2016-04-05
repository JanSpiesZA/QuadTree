int GridWidth = 100;
int GridHeight = 100;

int screenWidth = 640;
int screenHeight = 640;
int tileSize = 160;

int level = 2;

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
      
   allNodes.add(new Node(nodeX,nodeY, 0, allNodes.size()));    //Add new node to allNodes arrayList
      
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
    if (n.nodeType != 2)
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
    if (allNodes.get(k).nodeType == 1)
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
    if (allNodes.get(k).nodeType == 2)
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
          
  allNodes.add( new Node(startPos.x, startPos.y, 1, allNodes.size()));
  allNodes.add( new Node(goalPos.x, goalPos.y, 2, allNodes.size()));
  
  doQuadTree(0,0,maxHistogramX, maxHistogramY,level);
  println("\nNumber of allNodes: "+allNodes.size());        //Print the total number of nodes    
  nodeLink();  
  
  findPath();
  
  for (Node n: allNodes)
  {
   n.display();     
  }
}

//Code copied from: https://forum.processing.org/one/topic/rect-line-intersection-problem.html




/**
 * Determine whether a line intersects with a box. <br>
 * The box is represented by the top-left and
 * bottom-right corner coordinates.
 * @param lx0
 * @param ly0
 * @param lx1
 * @param ly1
 * @param rx0
 * @param ry0
 * @param rx1
 * @param ry1
 * @return true if they intersect else false
 */
boolean line_box_xyxy(float lx0, float ly0, float lx1, float ly1, float rx0, float ry0, float rx1, float ry1) {
  int out1, out2;
  float rectWidth = rx1 - rx0;
  float rectHeight = ry1 - ry0;
  if ((out2 = outcode(lx1, ly1, rx0, ry0, rectWidth, rectHeight)) == 0) {
    return true;
  }
  while ( (out1 = outcode (lx0, ly0, rx0, ry0, rectWidth, rectHeight)) != 0) {
    if ((out1 & out2) != 0) {
      return false;
    }
    if ((out1 & (OUT_LEFT | OUT_RIGHT)) != 0) {
      float x = rx0;
      if ((out1 & OUT_RIGHT) != 0) {
        x += rectWidth;
      }
      ly0 = ly0 + (x - lx0) * (ly1 - ly0) / (lx1 - lx0);
      lx0 = x;
    } else {
      float y = ry0;
      if ((out1 & OUT_BOTTOM) != 0) {
        y += rectHeight;
      }
      lx0 = lx0 + (y - ly0) * (lx1 - lx0) / (ly1 - ly0);
      ly0 = y;
    }
  }
  return true;
}

/**
 * Determine whether a line intersects with a box. <br>
 * The box is represented by the top-left corner coordinates
 * and the box width and height.
 * @param lx0
 * @param ly0
 * @param lx1
 * @param ly1
 * @param rx0
 * @param ry0
 * @param rWidth
 * @param rHeight
 * @return true if they intersect else false
 */
boolean line_box_xywh(float lx0, float ly0, float lx1, float ly1, float rx0, float ry0, float rWidth, float rHeight) {
  int out1, out2;
  if ((out2 = outcode(lx1, ly1, rx0, ry0, rWidth, rHeight)) == 0) {
    return true;
  }
  while ( (out1 = outcode (lx0, ly0, rx0, ry0, rWidth, rHeight)) != 0) {
    if ((out1 & out2) != 0) {
      return false;
    }
    if ((out1 & (OUT_LEFT | OUT_RIGHT)) != 0) {
      float x = rx0;
      if ((out1 & OUT_RIGHT) != 0) {
        x += rWidth;
      }
      ly0 = ly0 + (x - lx0) * (ly1 - ly0) / (lx1 - lx0);
      lx0 = x;
    } else {
      float y = ry0;
      if ((out1 & OUT_BOTTOM) != 0) {
        y += rHeight;
      }
      lx0 = lx0 + (y - ly0) * (lx1 - lx0) / (ly1 - ly0);
      ly0 = y;
    }
  }
  return true;
}

/*
 * Used by line - box intersection algorithm
 */
int outcode(float pX, float pY, float rectX, float rectY, float rectWidth, float rectHeight) {
  int out = 0;
  if (rectWidth <= 0) {
    out |= OUT_LEFT | OUT_RIGHT;
  } else if (pX < rectX) {
    out |= OUT_LEFT;
  } else if (pX > rectX + rectWidth) {
    out |= OUT_RIGHT;
  }
  if (rectHeight <= 0) {
    out |= OUT_TOP | OUT_BOTTOM;
  } else if (pY < rectY) {
    out |= OUT_TOP;
  } else if (pY > rectY + rectHeight) {
    out |= OUT_BOTTOM;
  }
  return out;
}

// Used in line-box intersection test
final int OUT_LEFT    = 1;
final int OUT_TOP     = 2;
final int OUT_RIGHT   = 4;
final int OUT_BOTTOM  = 8;

class Node
{ 
  int startIdx;
  int endIdx;
  int nodeID;      //A unique ID for each node
  int parentID = -1;    //
  float H = 0.0;
  float F = 0.0;
  float G = 0.0;
  
  // -----  //Calculate h first - dit is 'n konstante waarde van die node tot by die goal
  //add strarting node to closed list
  //add all currenlty linked nodes to open list if not there already
  //calc g score for all linked nodes in open list, from current node to linked node, use straight line distance for cost
  //    store g value in connected node - nie hierdie node nie.
  //    initial g value is straight line distance from current location
  //Calc f for each node: F = g + H
  //Pick node with lowest F score, this becomes current node
  //check all connected nodes to see if their G score is improved by going through current node first. 
      //If yes, then recalculate that nodes G score
  //Pick node with lowest F scrore and make it the current node
  
  
  float nodeXPos, nodeYPos;        //x and y location of node;
  
  boolean nodeActive = false;    //Active node used for path finding
  int nodeType = 0;              //0 = DEFAULT, 1 = START, 2 = GOAL
  
  ArrayList<Integer> nodeConnectedTo = new ArrayList<Integer>();
  

  
  Node(float _nodeXPos, float _nodeYPos)
  {        
    nodeType = 0;    //As opposed to START or GOAL
    nodeActive = false;
    
    nodeXPos = _nodeXPos;
    nodeYPos = _nodeYPos;
  }
  
  Node()
  {    
    nodeType = 0;    //As opposed to START or END
    nodeActive = false;
  }
  
  Node(int _nodeID)
  {    
    nodeType = 0;    //As opposed to START or END
    nodeActive = false;
    nodeID = _nodeID;
  }
  
  Node(float _nodeXPos, float _nodeYPos, int _nodeType)
  {    
    nodeType = _nodeType;    //As opposed to START or GOAL
    nodeActive = false;
    
    nodeXPos = _nodeXPos;
    nodeYPos = _nodeYPos;
  }
  
  Node(float _nodeXPos, float _nodeYPos, int _nodeType, int _nodeID)
  {    
    nodeType = _nodeType;    //As opposed to START or GOAL
    nodeActive = false;
    nodeID = _nodeID;
    
    nodeXPos = _nodeXPos;
    nodeYPos = _nodeYPos;
  }
  
//  Node(float _nodeXPos, float _nodeYPos, int _nodeType, int _nodeID)
//  {    
//    nodeType = 0;    //As opposed to START or GOAL
//    nodeActive = false;
//    nodeID = _nodeID;
//    
//    nodeXPos = _nodeXPos;
//    nodeYPos = _nodeYPos;
//  }
  
  
  void display()
  {
    strokeWeight(allNodesStrokeWeight);
    stroke(allNodesColor);
    
    switch (nodeType)
    {
      case 1:
        strokeWeight(1);
        fill(0,255,0);
        stroke(0);
        ellipse(nodeXPos, nodeYPos, 30,30);
        break;
        
      case 2:
        strokeWeight(1);
        fill(255,0,0);
        stroke(0);
        ellipse(nodeXPos, nodeYPos, 30,30);
        break;
        
      default:          
          ellipse(nodeXPos,nodeYPos, 10,10);   //Draws an ellipse to indicate node x,y        
    }

    textSize(30);    
    fill(0);
    textAlign(CENTER,CENTER);
    text(nodeID,nodeXPos,nodeYPos-20);
    textSize(15);
    text(int(H),nodeXPos+20,nodeYPos+20);  //Bottom Right - distance to GOAL    
    text(int(G),nodeXPos-20, nodeYPos+20);  //Bottom Left - from START with path values added    
    text(int(H+G),nodeXPos-20, nodeYPos-40);  //Top Left  -  sum of G and H
  }
}


class Tile
{
  int gravity = -1;    //Number of points in tile
  color gravityCol = color(gravity);
  //int[] arrayXY = {0,0};
  //int[] worldXY = {0,0};
  
  Tile()
  {
    gravityCol = color(150,200,150);
  }
  
  void clearGravity()
  {
    gravity = -1;
    gravityCol = color(150,200,150);
  }
  
  void update()
  {    
    switch(gravity)
    {
      case -1:
      {
        gravityCol = color(150,200,150);
        break;
      }
      case 1:
      {
        gravityCol = color(200,150,150);
        break;
      }
    }
  }
}
