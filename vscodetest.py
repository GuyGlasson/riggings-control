import maya.cmds as cmds
import functools


def box():

    cmds.curve (degree=1, point= [ (-1, 0, -1), (1, 0, -1), (1, 0, 1),
        (-1, 0, 1),  (-1, 0, -1), (-1, 2, -1), (1, 2, -1), (1, 0, -1), 
        (1, 2, -1), (1, 2, 1),(1, 0, 1), (1, 2, 1), (-1, 2, 1),
        (-1, 0, 1),(-1, 2, 1), (-1, 2, -1) ])


def Sphere():
    
    circleX = cmds.circle (nr=(1,0,0))
    circleY = cmds.circle (nr=(0,1,0))
    circleZ = cmds.circle (nr=(0,0,1))

    circleList =  circleY + circleZ
    
    cmds.select (circleList, r=True)
    circleShape = cmds.ls(sl=True, dag=True, ap=True, s=True)
    cmds.parent (circleShape, circleX, r=True,  s=True ) 
    cmds.delete(circleList)


def diamond():

    cmds.curve (degree=1, point=[(0, 0, 1), (0, -2, 0), (0.866025, 0, 0.5), (0, 2, 0),
        (0.866025, 0, -0.5), (0, -2, 0), (0, 0, -1), (0, 2, 0), (-0.866025, 0, -0.5), 
        (0, -2, 0), (-0.866025, 0, 0.5), (0, 2, 0), (0, 0, 1), (-0.866025, 0, 0.5), 
        (-0.866025, 0, -0.5), (0, 0, -1), (0.866025, 0, -0.5),(0.866025, 0, 0.5),
        (0, 0, 1)])

def cross():
    
    cmds.curve (degree=1, point= [(0, 0, -1), (1, 0, -2), (2, 0, -1), (1, 0, 0), (2, 0, 1), (1, 0, 2),
    (0, 0, 1), (-1, 0, 2), (-2, 0, 1), (-1, 0, 0), (-2, 0, -1), (-1, 0, -2), (0, 0, -1)])

def twist_joint(joinXField, joinTranXField, joinNameField):

    joinName = cmds.textFieldGrp( joinNameField, query=True, text=True)
    joinX = cmds.intField( joinXField, query=True, value=True )
    joinTranX = cmds.floatField( joinTranXField, query=True, value=True )
    

    joinTranX /=float(joinX-1) 
    joinTranXoriginal = joinTranX
    joinList=[] 

    #Make joint base on the number in joinXField
    cmds.select(clear=True) 
    for i in range(0,joinX):
        if i==0:
            join=cmds.joint( p=(0, 0, 0),name=joinName+"_joint_"+str(i) )
            
            joinList.append(join)
            

        else:
            join=cmds.joint( p=(joinTranX, 0, 0),name=joinName+"_joint_"+str(i) )
            joinTranX+=joinTranXoriginal
            joinList.append(join)

        #when the base joint are made this if make IK spline solver and groups
        if i==joinX-1:
            
            masterGroup=cmds.group( name = joinName+"_null",empty=True)
            cmds.parent( joinList[0], masterGroup)
        
            ik = cmds.ikHandle( sj=joinList[0], ee=joinList[-1], solver='ikSplineSolver')

            cmds.parent( ik[0] , masterGroup)

            noInheritsTransform = cmds.group(ik[2] , name = joinName+"_noInheritsTransform")
            cmds.setAttr(noInheritsTransform+'.inheritsTransform', 0)
            
            
            #add the variables name to the start of the IK obj 
            ikHands =[]
            for obj in ik:
                cmds.select(obj, r=True)
                rename=cmds.rename( joinName+"_"+obj)
                ikHands.append(rename)
                cmds.select(clear=True)
                 
            #Get the start and end joint tranforms 
            sj = cmds.xform(joinList[0],q=1,ws=1,rp=1)
            ej = cmds.xform(joinList[-1],q=1,ws=1,rp=1)
            
            

            #making the control joints that will be skin bind to the curve 
            cmds.select(clear=True)
            startJoin=cmds.joint(p=(sj[0], sj[1], sj[2],), name= joinName+"_start_ctrl_Joint")
            cmds.setAttr(startJoin+'.radius', 4)
            startJoinGroup=cmds.group(startJoin, name = startJoin+"_null")

            cmds.select(clear=True)
            endJoin=cmds.joint(p=(ej[0], ej[1], ej[2],), name= joinName+"_end_ctrl_Joint")
            cmds.setAttr(endJoin+'.radius', 4)
            endJoinGroup=cmds.group(endJoin, name = endJoin+"_null")

            cmds.select(clear=True)
            midJoin=cmds.joint( name= joinName+"_mid_ctrl_Joint")
            cmds.setAttr(midJoin+'.radius', 4)
            midJoinGroup=cmds.group(midJoin, name = midJoin+"_null")
            cmds.pointConstraint( startJoin, endJoin, midJoinGroup, maintainOffset=False )
            cmds.aimConstraint( endJoin, midJoinGroup)

            #group control joint under the top node
            cmds.parent(startJoinGroup, midJoinGroup, endJoinGroup, masterGroup)

            #skining the control bone to the curve 
            cmds.skinCluster(ikHands[-1], startJoin, midJoin, endJoin)
            cmds.select(clear=True)

            #build strech rig
            #give the curver history info 
            curveHist = cmds.arclen( ikHands[-1], ch=True )
            curveLength = cmds.getAttr(curveHist+".arcLength" )

            #Build a plus minus average node to be used to average length 
            averageLength = cmds.shadingNode ( 'multiplyDivide', name = joinName+"_average_Length", asUtility =True)
            cmds.setAttr(averageLength+".operation", 2)
            cmds.setAttr(averageLength+".input2X", curveLength)
            
            averageTranslate = cmds.shadingNode ( 'multiplyDivide', name = joinName+"_average_Translate", asUtility =True)
            joinTranslateX = cmds.getAttr(joinList[1]+".translateX" )
            cmds.setAttr(averageTranslate+".input2X", joinTranslateX)
            
            
            
            #making connetons between nodes 
            cmds.connectAttr (curveHist+".arcLength", averageLength+".input1X")
            cmds.connectAttr (averageLength+".outputX", averageTranslate+".input1X")

            for join in joinList[1:]:
               
                cmds.connectAttr(averageTranslate+".outputX", join+".translateX" )

            
            
           
            

            #for twist will be add later
            #cmds.setAttr(ikHands[0]+'.dTwistControlEnable', 1)
            #cmds.setAttr(ikHands[0]+'.dWorldUpType', 2)


            

def null(): 
#Simple null script

    #Make a list from selecton
    myls=cmds.ls(sl=True)
    #runs through each select obj and give them a group
    for obj in myls:

        if obj.endswith("null"):
            newGroup=cmds.group(obj)
            groupList=cmds.rename(newGroup, obj+"_offSet")

        elif obj.endswith("offSet"): 
            None

        else:
            newGroup=cmds.group(obj)    
            groupList=cmds.rename(newGroup, obj+"_null")
    

 

def setColour(Colour):
    
    objShape=cmds.ls(sl=True, dag=True, ap=True, s=True)
    
    
    for obj in objShape: 
        cmds.setAttr(obj+'.overrideEnabled',1)
        cmds.setAttr(obj+'.overrideColor',Colour)

        



def showWindow():
    name = "RiggingScript"

    if cmds.window(name, query=True, exists=True):
        cmds.deleteUI(name)
        
    cmds.window(name)
    cmds.showWindow()
    
    column = cmds.columnLayout()
    
    cmds.gridLayout(numberOfColumns=1, cellWidthHeight=(300, 20))
    cmds.text("Controls",font="boldLabelFont", align='center',backgroundColor=(0,0,0))

    cmds.setParent(column)
    #walk.png polyPyamid.png
    cmds.gridLayout(numberOfColumns=4, cellWidthHeight=(75, 70))
    
    cmds.iconTextButton( style='textOnly', label='Box',backgroundColor=(0.5, 0.5, 0.5), command=box)
    cmds.iconTextButton( style='textOnly', label='Sphere',backgroundColor=(0.5, 0.5, 0.5), command=Sphere)
    cmds.iconTextButton( style='textOnly', label='Diamond',backgroundColor=(0.5, 0.5, 0.5), command=diamond)
    cmds.iconTextButton( style='textOnly', label='Cross',backgroundColor=(0.5, 0.5, 0.5), command=cross)
    
   
   

    cmds.setParent(column)
    cmds.gridLayout(numberOfColumns=1, cellWidthHeight=(300, 20))
    cmds.text("Colour",font="boldLabelFont", align='center',backgroundColor=(0,0,0))

  
    cmds.setParent(column)
    #Colour button that pass a value onto setColour
    cmds.gridLayout(numberOfColumns=7, cellWidthHeight=(43, 40))
    #cmds.rowLayout (numberOfColumns=7 )
    cmds.iconTextButton(image="red_icon.png", w=40, h=40, command=lambda:setColour(13))
    cmds.iconTextButton(image="darkRed_icon.png", w=40, h=40, command=lambda:setColour(4))
    cmds.iconTextButton(image="green_icon.png", w=40, h=40, command=lambda:setColour(14))
    cmds.iconTextButton(image="yellow_icon.png", w=40, h=40, command=lambda:setColour(17))
    cmds.iconTextButton(image="purple_icon.png", w=40, h=40, command=lambda:setColour(9))
    cmds.iconTextButton(image="darkBlue_icon.png", w=40, h=40, command=lambda:setColour(5))
    cmds.iconTextButton(image="blue_icon.png", w=40, h=40, command=lambda:setColour(6))
    
    cmds.setParent(column)
    cmds.gridLayout(numberOfColumns=1, cellWidthHeight=(300, 40))
    cmds.iconTextButton(style='textOnly', label='Null', backgroundColor=(0.5, 0.5, 0.5), command=null)

    cmds.setParent(column)
    cmds.gridLayout(numberOfColumns=1, cellWidthHeight=(300, 20))
    cmds.text("Twist Rig",font="boldLabelFont", align='center',backgroundColor=(0,0,0))

    cmds.setParent(column)
    cmds.rowColumnLayout( numberOfColumns=4, columnWidth=[ (1,5), (2,90), (3,110), (4,80) ]  )
    cmds.separator( h=10, style='none' )
    cmds.checkBox( label='strech rig' )
    cmds.checkBox( label='Advanced twist' )
    cmds.checkBox( label='Controls' )
    
    cmds.setParent(column)
    cmds.rowColumnLayout( numberOfColumns=3, columnWidth=[ (1,5), (2,143), (3,138) ] )
    cmds.separator( h=10, style='none' )
    cmds.text(label='Number of Joints', align='left')
    joinXField = cmds.intField(value=3)
    
    
    cmds.setParent(column)
    cmds.rowColumnLayout( numberOfColumns=4, columnWidth=[ (1,5), (2,140), (3,3), (4,138) ] )
    cmds.separator( h=10, style='none' )
    cmds.text(label='length of rig', align='left')
    cmds.separator( h=10, style='none' )
    joinTranXField = cmds.floatField(value=5,)
    

    
    cmds.setParent(column)
    cmds.rowColumnLayout( numberOfColumns=3, columnWidth=[ (1,5), (2,280), (1,5) ] )
    cmds.separator( h=10, style='none' )
    joinNameField=cmds.textFieldGrp( label='Enter Name', text='Editable', columnAlign2=["left", "left"], editable=True )
 
    cmds.setParent(column)
    cmds.gridLayout(numberOfColumns=1, cellWidthHeight=(300, 40))
    
    cmds.iconTextButton(style='textOnly', label='Build', backgroundColor=(0.5, 0.5, 0.5), 
                        command=functools.partial(twist_joint, joinXField, joinTranXField, joinNameField) )
   
     
     

   
   

    

    
    
showWindow()    

    
    
