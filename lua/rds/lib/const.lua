--@@常量参数

local const={}

const.PATHNAME="path_db"
--机械爪的动作 开|闭|不变
const.action={
    open=0,
    close=1,
    none=2
}

const.frameID={
    world=-1,
    base=0,
    self=1
}

--通信
const.COM={
    --轨迹命令:uint8
    TRAJ_CMD_NEW=1,
    TRAJ_CMD_START=2,
    TRAJ_CMD_PAUSE=3,
    TRAJ_CMD_STOP=0,

    --机器人模式:uint8
    ROBO_MODE_AUTO=  1, --自动模式
    ROBO_MODE_MANNUL=  2,--手动模式
    ROBO_MODE_UNKNOW=  3,--未知模式

    --机器人状态:uint8
    ROBO_STATUS_TRUE=255,
    ROBO_STATUS_FALSE=0,

    --机器人错误码 uint16
    ROBO_ERRORCODE_NONE=0,  --无错误
    ROBO_ERRORCODE_ALARM=10, --报警错误
    MAX_PTS_NUMBER=100000,  --最大的轨迹点的数据


    --ADS命令 初始化|读取|写入|读写|销毁
    ADS_CMD_INIT=0,
    ADS_CMD_READ=1,
    ADS_CMD_WRITE=2,
    ADS_CMD_READWRITE=3,
    ADS_CMD_DESTROY=4,


    --路径规划的命令
    PLAN_CMD_NEW=0,
    PLAN_CMD_STOP=1



}

const.DB={
    UI="uiData", --界面保存参数
    TRAJ="trajData",  --计算的轨迹路径
}

const.TOPICS={
    ADSCMD="adscmd",  --ADS控制命令,初始化|关闭
    TRAJCMD="trajcmd", --发出开始执行轨迹命令
    PLANCMD="plancmd", --发出开始规划的命令
    ROBOSTATES="robostates", --采集到的关节数据/机器人状态
    PLANEDPATH="planedpath" --规划出来的轨迹数据
}

const.UI={
    comboSel=1000,
    editPath=1201,
    buttonPath=1202,
    editTarget=1203,
    editIndex=1300,
    editStep=1400,
    remoteID=2000,
    remoteIP=2001,
    localID=2002,
    editWriteAddr=2003,
    editReadAddr=2004,
    editDriverPowered=3001,
    editEStopped=3002,
    editInError=3003,
    editInMotion=3004,
    editImotionPossible=3005,
    editErrorCode=3006,
    treePathStatus=5000,
    groupManul=8000,
    groupAuto=8001,
    treeJointStatus=9000,
    checkRead=9001,
    checkSync=9002,
    labelInMotion=9003,
    plotCurve=9600
}

return const