#import numpy

#Script to automate translating all of the pixel coordinate data for the 1 meter primary map into ACS

#Stolen from 2021 (which in turn was stolen from docs)
def PCStoACS(x, y):
    halfResX = 1920 / 2
    halfResY = 1080 / 2

    ACSX = (x - halfResX) / halfResX
    ACSY = (y - halfResY) / halfResY
    ACSCOORDS = [ACSX, ACSY]

    return ACSCOORDS

#I'm not ordering the points by hand
#Puts the points in an order where OpenCV will stop complaining (i.e. CCW instead of 'whatever')
#this is a utility script I take full license to make it as awful as I please
def SortQuad(quad):
    outputquad = []
    outputquad.append(quad[0])
    outputquad.append(quad[1])
    outputquad.append(quad[3])
    outputquad.append(quad[2])
    return outputquad

PCSX = [
    [738, 798, 738, 798], [871, 1154, 871, 1154], [1248, 1308, 1248, 1308],
    [652, 713, 652, 713], [844, 1195, 844, 1195], [1333, 1394, 1333, 1394],
                          [977, 1093, 977, 1093],
    [288, 768, 288, 768], [835, 1239, 835, 1239], [1278, 1758, 1278, 1758]
    ]

PCSY = [
    [186, 186, 258, 258], [212, 212, 279, 279], [186, 186, 258, 258],
    [307, 307, 404, 404], [319, 319, 414, 414], [307, 307, 404, 404],
                          [436, 436, 549, 549],
    [649, 649, 949, 949], [645, 645, 950, 950], [649, 649, 949, 949]
    ]

ACSOUT = []
RELACSOUT = []
RELACSORIGIN = []   #origin of apriltag in absolute position; used as a straight linear 
                    #transform for everything else because ACS is linear

#go to acs
for index in range(10):
    QUADPOINTS = []
    for quadindex in range(4):
        QUADPOINTS.append(PCStoACS(PCSX[index][quadindex], PCSY[index][quadindex]))
    ACSOUT.append(QUADPOINTS)

#print ACSOUT
print("ABSOLUTE ACS==========================================")
for index in range(10):
    print(index)
    print("[")
    for quadindex in range(4):
        print(ACSOUT[index][quadindex])
    print("]")

#relative-to-middle-of-apriltag ACS
print("UNTRANSFORMED===============")
print(ACSOUT[6])
RELACSORIGIN.append( (ACSOUT[6][0][0] + ACSOUT[6][1][0]) / 2 )
RELACSORIGIN.append( (ACSOUT[6][0][1] + ACSOUT[6][2][1]) / 2 )
#nobody's going to read this code right
print("origin at " + str(RELACSORIGIN))
for index in range(10):
    RELACSOUTPOINTS = []
    for quadindex in range(4):
        #god it's so nice not needing to worry about null pointers
        #for all these terribly written array operations
        RELACSOUTPOINTS.append([ACSOUT[index][quadindex][0] - RELACSORIGIN[0],
                                 ACSOUT[index][quadindex][1] - RELACSORIGIN[1]])
    RELACSOUT.append(RELACSOUTPOINTS)
print("RELATIVE ACS===========================================")
for index in range(10):
    print(index)
    print("[")
    for quadindex in range(4):
        print(RELACSOUT[index][quadindex])
    print("]")

RELACSSORTED = []
#sort points
for index in range(10):
    RELACSSORTED.append(SortQuad(RELACSOUT[index]))

print("RELATIVE ACS SORTED=====================================")
for index in range(10):
    print(index)
    print("[")
    for quadindex in range(4):
        print(RELACSSORTED[index][quadindex])
    print("]")