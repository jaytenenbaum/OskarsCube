import numpy as np
import os
import random
import stl
from stl import mesh
import sys
from datetime import datetime
import numpy as np
from numpy import array
import sys
(sizex,sizey,sizez)=(5,5,5)
minimalPathLength = 0
cubeLocation = "inputs/cube.stl"
class Grid:
    def __init__(self, (sizex,sizey,sizez)):
        self.size = (sizex,sizey,sizez)
        self.grid = np.array([[[0 for i in range(sizez)] for j in range(sizey)] for j in range(sizex)])#a 3d-grid of the maze
        self.xygrid = np.array([[0 for i in range(sizey)] for j in range(sizex)])# the projection on xy axis
        self.xyBrokenWalls = set()#no broken wall yet
        self.xzgrid = np.array([[0 for i in range(sizez)] for j in range(sizex)])# the projection on xz axis
        self.xzBrokenWalls = set()#no broken wall yet
        self.yzgrid = np.array([[0 for i in range(sizez)] for j in range(sizey)])# the projection on yz axis
        self.yzBrokenWalls = set()#no broken wall yet
    def __setitem__(self, key, item):
        """update the projections based on the update of the major grid"""
        self.grid[key] = item
        if item == 1:
            self.xygrid[key[0],key[1]] = 1
            self.xzgrid[key[0],key[2]] = 1
            self.yzgrid[key[1],key[2]] = 1
    def __getitem__(self, key):
        return self.grid[key]
def getSingleEnlargedGrid(abgrid, abBrokenWalls,(enlargedSizea,enlargedSizeb)):
    """inflate a specific 2d-proj, by filling all walls from all edges, and removing the walls of edges we've removed"""
    abgrid = np.array([[0 for i in range(enlargedSizeb)] for j in range(enlargedSizea)])
    # fill all walls as black
    for i in range(enlargedSizea):
        for j in range(enlargedSizeb):
            if i%2==1 or j%2==1:
                abgrid[i,j]=1
    # remove removed walls
    for ((a,b),(c,d)) in abBrokenWalls:
        if a==c:
            abgrid[2*a][2*min(b,d)+1]=0
        else:#b==d
            abgrid[2*min(a,c)+1][2*b]=0
    return abgrid
def createEnlargedGrids(grd):
    """take the grid from the dfs with the edge information, and inflate the grid such that each full edge becomes now a full cube."""
    (enlszx,enlszy,enlszz) = (grd.size[0]*2-1,grd.size[1]*2-1,grd.size[2]*2-1)# inflation is 2 times minus 1, e.g., when original maze's size is 2, there is 1 additional wall=>3
    grdxy = getSingleEnlargedGrid(grd.xygrid, grd.xyBrokenWalls,(enlszx,enlszy))
    grdxz = getSingleEnlargedGrid(grd.xzgrid, grd.xzBrokenWalls,(enlszx,enlszz))
    grdyz = getSingleEnlargedGrid(grd.yzgrid, grd.yzBrokenWalls,(enlszy,enlszz))
    return grdxy,grdxz,grdyz
def cubeData(scale = (1,1,1),trans = (0,0,0)):
    """return the data of a cube from cubeLocation, and scale it by input, and then translate by input"""
    scx,scy,scz = scale
    cubeMesh = mesh.Mesh.from_file(cubeLocation)
    cubeData = cubeMesh.data
    cubeMesh.data["vectors"]*=scale#scale by given scale
    cubeMesh.translate(list(trans))#translate (the package expects a list instead of a tuple, therefore the cast)
    return cubeMesh.data
    
def legalNeighborsAxisDir(vertex, (sizex,sizey,sizez)):
    """return all legal neighbors by moving perpendicular to each axis, and assuring we're still in the bounds of the cube."""
    x=vertex[0]
    y=vertex[1]
    z=vertex[2]
    neighbors = [(x - 1, y,z), (x, y + 1,z), (x + 1, y,z), (x, y - 1,z),(x, y,z+1),(x, y ,z-1)]#6 possible neighbors: above,below,right,left,inner,outer
    inBoundNeighbors = [(x,y,z) for (x,y,z) in neighbors if x>=0 and x<sizex and y>=0 and y<sizey and z>=0 and z<sizez]# filter neighbors in bounds of the cube
    return inBoundNeighbors
def isRemoved(removedSet,(a,b),(x,y)):
    """returns whether the edge between 2d-proj indices (a,b),(x,y) was removed already"""
    return ((a,b),(x,y)) in removedSet or ((x,y),(a,b)) in removedSet

def isGoodNewNeighbor((x,y,z),grid,(origx,origy,origz)):
    """a new neighbor is good if we haven't reached him yet, and for each ab-projection:
    1. it hasn't been explored yet (so we can explore it and break the wall)
    or
    2. we travel perpendicular to it [and therefore the cross doesn't move relatively to it, and won't collide with its wall]
    or
    3. we've already broken the wall in 2d-proj and we traverse it again [and therefore no collision with it when moving from orig to (x,y,z)]
    """
    if grid[x,y,z]!=0:
        return False
    return     (grid.xygrid[x,y]==0 or z!=origz or isRemoved(grid.xyBrokenWalls, (x,y),(origx,origy)))\
           and (grid.xzgrid[x,z]==0 or y!=origy or isRemoved(grid.xzBrokenWalls, (x,z),(origx,origz)))\
           and (grid.yzgrid[y,z]==0 or x!=origx or isRemoved(grid.yzBrokenWalls, (y,z),(origy,origz)))
def discoverNeighbors(vertex,grid):
    """discover the new neighbors to add to end of dfs stack"""
    (vx,vy,vz) = vertex
    #possible new neightbors are triplets of locations beside vertex (at most 6), and that it's legal to break the wall to (they are good)
    newNeighbors = [(x,y,z) for (x,y,z) in legalNeighborsAxisDir(vertex, grid.size) if isGoodNewNeighbor((x,y,z),grid,vertex)]
    for (a,b,c) in newNeighbors:
        #mark as discovered
        grid[(a,b,c)]=1
        #exactly one happens - break the relevant walls in the 2d sides [or nothing if already broken, as grid.*BrokenWalls is a set]
        if a!=vx:
            grid.xyBrokenWalls.add(((vx,vy),(a,b)))
            grid.xzBrokenWalls.add(((vx,vz),(a,c)))
        if b!=vy:
            grid.xyBrokenWalls.add(((vx,vy),(a,b)))
            grid.yzBrokenWalls.add(((vy,vz),(b,c)))
        if c!=vz:
            grid.xzBrokenWalls.add(((vx,vz),(a,c)))
            grid.yzBrokenWalls.add(((vy,vz),(b,c)))
    random.shuffle(newNeighbors)#shuffle between the new neifhbors to explore
    return newNeighbors#return the new possible neighbors to explore
def getSolutionPath(prevMap, initialPoint, farthest):
    """given the map which for each vertex maps to the vertex that discovered it, and the initial and farthest point throughout the dfs, it returns the dfs path from
    initialPoint to farthest point"""
    x=farthest# start from last
    path = [farthest]
    # build path from last to first
    while x!= initialPoint:
        x=prevMap[x]
        path.append(x)
    return path[::-1]#reverse path
def dfs(grid,initialPoint):
    """create the maze in dfs manner. we start in the point (0,0,0) and as long as stack isn't clear,
     we develop the neighbors of the next vertex and add them to the stack"""
    maxDepth = -1
    stack = [(initialPoint,0)]
    prevMap = dict()
    grid[initialPoint]=1#mark that we started from initialPoint
    #a list of pairs, maintaining the new vertices we've explored, and for each one a flag of whether he found new neighbors [this is for outputting the solution path]
    while stack!=[]:#as long as we can still explore
        (vertex,depth) = stack.pop()#take top
        if depth>maxDepth:
            farthest=vertex
            maxDepth=depth
        newNeighbors = discoverNeighbors(vertex,grid)#explore it
        for x in newNeighbors:
            prevMap[x]=vertex
        stack+=[(x,depth+1) for x in newNeighbors]#add new neghbors for future exploring
    solutionPath = getSolutionPath(prevMap, initialPoint, farthest)
    #solutionPath,streak,lastPoint = getSolutionPath(verticesTravelledAndFoundNewNeighbors)#get the path of the solution
    return grid,len(solutionPath)-1,farthest,solutionPath

def createGrid((sizex,sizey,sizez), minimumLength = 0):
    """create a grid by initing it and creating the maze in a dfs manner"""
    #set the maze initial point as center of top z face (this is good for printing, otherwise the cube floats over cross)
    initialPoint = (int(sizex/2),int(sizey/2),sizez-1)
    maxPathLength = 0
    for i in range(1000):
        grd,streak,lastPoint,sequence = dfs(Grid((sizex,sizey,sizez)),initialPoint)#create a random maze, hoping it satisfies minimum length
        if(streak>=minimumLength):#the length of the path is above minimum length
            return grd,streak,lastPoint,sequence
        else:#update maximal length until now
            maxPathLength = max(maxPathLength,streak)
    print("Couldn't generate a long enough solution path.\nMaximum length out of 1,000 tries was " + str(maxPathLength)+", which is smaller than "+str(minimumLength)+"!")
    sys.exit(0)
def voxelizeLocationsMesh(locations):
    """receives a list of tuples [(x1,y1,z1),...] where to build voxels, and creates a mesh containing those relevant cubes"""
    listOfVoxels = []
    for (a,b,c) in locations:
        listOfVoxels.append(cubeData(trans = (a,b,c)))        
    outputMesh = mesh.Mesh(np.concatenate(listOfVoxels))
    return outputMesh
def gridsToVoxelLocations(grdxy,grdxz,grdyz):
    """receives as an input the three grids of the 3 planes, and returns a list of the locations where to create the voxels of the maze itself"""
    locs = set()
    #lengths of axes
    (lnx,lny,lnz)=(len(grdxy),len(grdxy[0]),len(grdxz[0]))

    #draw the inner maze for all 6 planes [drawing 2 planes for each of {xy,xz,yz}]
    for i in range(lnx):
        for j in range(lny):
            if grdxy[i,j]!=0:
                #both planes of cube
                locs.add((i+1,j+1,0))
                locs.add((i+1,j+1,lnz+1))
    for i in range(lnx):
        for j in range(lnz):
            if grdxz[i,j]!=0:
                #both planes of cube
                locs.add((i+1,0,j+1))
                locs.add((i+1,lny+1,j+1))
    for i in range(lny):
        for j in range(lnz):
            if grdyz[i,j]!=0:
                #both planes of cube
                locs.add((0,i+1,j+1))
                locs.add((lnx+1,i+1,j+1))

    ##create frame (may contain multiples in corners, but locs is a set so no harm done)
    for i in range(0,lnx+2):#yz frames
        locs.add((i,0,0))
        locs.add((i,lny+1,0))
        locs.add((i,0,lnz+1))
        locs.add((i,lny+1,lnz+1))
    for i in range(0,lny+2):#xz frames  
        locs.add((0,i,0))
        locs.add((lnx+1,i,0))
        locs.add((0,i,lnz+1))
        locs.add((lnx+1,i,lnz+1))
    for i in range(0,lnz+2):#xy frames   
        locs.add((0,0,i))
        locs.add((0,lny+1,i))
        locs.add((lnx+1,0,i))
        locs.add((lnx+1,lny+1,i))
    return list(locs)
def middle((a,b,c),(x,y,z)):
    """returns the average integer value of two given 3d-points"""
    return (int((a+x)/2),int((b+y)/2),int((c+z)/2))
def describe(ans):
    """describes the answer sequence as a string"""
    return "sequence of "+str(len(ans)-1)+" steps from "+str(ans[0])+" to "+str(ans[-1])+"\n"+"sequence is: "+str(ans)
def get3dvoxelsFromSequence(sequence):
    """receives a sequence of moves, and returns the set of voxels of the sequence inside the cube mesh, needed for the file 'voxelizedWithSolution'
    Note: The input is given as original maze coordinates, and not as inflated cube and therefore need a scale of 2."""
    rescaledseq = [(2*x,2*y,2*z) for (x,y,z) in sequence]#sequence in extended maze
    ans = [rescaledseq[0]]
    #iterate over the sequence, maintaining the previous rescaled location, and create the ans which is the list of the sequence
    prev = rescaledseq[0]
    for i in range(1,len(sequence)):
        #add cube in the middle of current and previous [this is necessary since we've done the inflation so the middle cubes are lost, i.e., in inflated maze we need to draw 2 times more cubes]
        ans.append(middle(rescaledseq[i], prev))
        ans.append(rescaledseq[i])
        prev = rescaledseq[i]
    #text describing the solution for output to standard output and file
    outputTxt = "In inflated maze (original scaled by 2):\n"+describe(ans) +"\n\nAlthought, in original deflated maze:\n"+describe(sequence)
    print(outputTxt)
    open("outputs/outputSequence.txt",'w').write(outputTxt)
    return ans
def addCrossMesh(voxelizedMesh,(sizex,sizey,sizez),(centerx,centery,centerz),outputFileName):
    """receives the voxelized mesh, and adds the cross at the start point [created by scaling and stranslating a cube]."""
    deflationConst = 0.8 # the relative height of the cross to the cube size of the maze [ideally should be 0.999 for the cross to just slide in the maze]
    cross = []
    newCrossx = cubeData(scale=(2*sizex-1,deflationConst,deflationConst),\
                         trans=(centerx-(sizex-1)+(1-deflationConst)/2,centery-(1-deflationConst)/2,centerz))#x stick of cross
    cross.append(newCrossx)
    
    newCrossy = cubeData(scale=(deflationConst,2*sizey-1,deflationConst),\
                         trans=(centerx+(1-deflationConst)/2,centery+(sizey-1)-(1-deflationConst)/2,centerz))#y stick of cross
    cross.append(newCrossy)
    
    newCrossz = cubeData(scale=(deflationConst,deflationConst,2*sizez-1),\
                         trans=(centerx+(1-deflationConst)/2,centery-(1-deflationConst)/2,centerz-(sizez-1)))#z stick of cross
    cross.append(newCrossz)        
    
    outputMesh = mesh.Mesh(np.concatenate(cross+[voxelizedMesh.data.copy()]))#union cross sticks to a single mesh
    return outputMesh
 
def addStartAndEndAndSave(crossMesh,(sizex,sizey,sizez), (startx,starty,startz),(endx,endy,endz), outputFileName,grdxy,grdxz,grdyz):
    """add the start and end markings on the cube [4 tall outer cubes enclose the start, and 4 short outer cubes enclose the end]"""
    (startx,starty,startz)=(startx*2,starty*2,startz*2)#scaled maze factor
    (endx,endy,endz)=(endx*2,endy*2,endz*2)#scaled maze factor
    additionals = []
    ##adding start
    for (x,y) in [(startx+1,starty+1),(startx+1,starty-1),(startx-1,starty+1),(startx-1,starty-1)]:#add 4 tall cubes surrounding start location in x-y face
        additionals.append(cubeData(trans=(x+1,y+1,sizez+1)))
    for (x,z) in [(startx+1,startz+1),(startx+1,startz-1),(startx-1,startz+1),(startx-1,startz-1)]:#add 4 tall cubes surrounding start location in x-z face
        additionals.append(cubeData(trans=(x+1,sizey+1,z+1)))
    for (y,z) in [(starty+1,startz+1),(starty+1,startz-1),(starty-1,startz+1),(starty-1,startz-1)]:#add 4 tall cubes surrounding start location in y-z face
        additionals.append(cubeData(trans=(sizex+1,y+1,z+1)))
    ##adding end
    for (x,y) in [(endx+1,endy+1),(endx+1,endy-1),(endx-1,endy+1),(endx-1,endy-1)]:#add 4 short cubes surrounding start location in x-y face
        additionals.append(cubeData(scale=(1,1,0.5),trans=(x+1,y+1,sizez+1)))
    for (x,z) in [(endx+1,endz+1),(endx+1,endz-1),(endx-1,endz+1),(endx-1,endz-1)]:#add 4 short cubes surrounding start location in x-z face
        additionals.append(cubeData(scale=(1,0.5,1),trans=(x+1,sizey+0.5,z+1)))
    for (y,z) in [(endy+1,endz+1),(endy+1,endz-1),(endy-1,endz+1),(endy-1,endz-1)]:#add 4 short cubes surrounding start location in y-z face
        additionals.append(cubeData(scale=(0.5,1,1),trans=(sizex+1,y+1,z+1)))
        
    withStartMesh = mesh.Mesh(np.concatenate(additionals+[crossMesh.data.copy()]))#concatenate into a single mesh
    withStartMesh.save(outputFileName,mode = stl.Mode.ASCII)
def main():
    #create the outputs folder
    if not os.path.exists("outputs"):
        os.makedirs("outputs")
    tstart = datetime.now()#start timing
    grd,streak,lastPoint,sequence = createGrid((sizex,sizey,sizez),minimalPathLength)#create the maze!
    tend = datetime.now();print("creating grid took "+str(tend - tstart));tstart=tend#print timing and reset timing
    grdxy,grdxz,grdyz = createEnlargedGrids(grd)#inflate the maze (as described in docx, we inflate each maze wall into a cube for the maze to be printable)
    grdstovoxels = gridsToVoxelLocations(grdxy,grdxz,grdyz)# get locations of the voxels of the maze itself
    voxelizedMesh = voxelizeLocationsMesh(grdstovoxels)# save the maze as voxels
    #add the cross at the beginning
    crossMesh = addCrossMesh(voxelizedMesh,(grd.size[0]*2,grd.size[1]*2,grd.size[2]*2),(2*sequence[0][0]+1,2*sequence[0][1]+1,2*sequence[0][2]+1), "outputs/voxelizedWithCross.stl")
    #add markers of start/end of maze (surrounding by 4 tall/short cubes)
    addStartAndEndAndSave(crossMesh,(grd.size[0]*2,grd.size[1]*2,grd.size[2]*2), sequence[0],sequence[-1],"outputs/printableMaze.stl",grdxy,grdxz,grdyz)
    #add an output of the solution inside the maze frame
    voxelizeLocationsMesh(grdstovoxels+[(x+1,y+1,z+1) for (x,y,z) in get3dvoxelsFromSequence(sequence)]).save("outputs/solutionMaze.stl",mode = stl.Mode.ASCII)
    tend = datetime.now();print("outputs took "+str(tend - tstart))#print time passed
    print("Done!!!")
    
if __name__ == "__main__":
    if len(sys.argv[1:])==0:
        additionalArguments=False
    else:
        additionalArguments=True
    argv=sys.argv[1:]
    if additionalArguments:
        sizex = int(argv[0])
        sizey = int(argv[1])
        sizez = int(argv[2])
        if len(argv)>3:
            minimalPathLength = int(argv[3])
        else:
            minimalPathLength=0
    main()
