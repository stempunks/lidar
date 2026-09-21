let CMD_SETMODE                 = 1
let CMD_ALLData                 = 2
let CMD_FIXED_POINT             = 3
let CMD_LINE                    = 4
let CMD_LIST                    = 5
let CMD_AVOID_OBSTACLE          = 6
let CMD_CONFIG_AVOID            = 7
let CMD_OBSTACLE_DISTANCE       = 8
let CMD_END                     = CMD_OBSTACLE_DISTANCE
let STATUS_SUCCESS              = 0x53   
let STATUS_FAILED               = 0x63  
let IIC_MAX_TRANSFER            = 32     
let I2C_ACHE_MAX_LEN            = 32
let DEBUG_TIMEOUT_MS            = 8000
let ERR_CODE_NONE               = 0x00 
let ERR_CODE_CMD_INVAILED       = 0x01 
let ERR_CODE_RES_PKT            = 0x02 
let ERR_CODE_M_NO_SPACE         = 0x03 
let ERR_CODE_RES_TIMEOUT        = 0x04 
let ERR_CODE_CMD_PKT            = 0x05 
let ERR_CODE_SLAVE_BREAK        = 0x06 
let ERR_CODE_ARGS               = 0x07 
let ERR_CODE_SKU                = 0x08  
let ERR_CODE_S_NO_SPACE         = 0x09 
let ERR_CODE_I2C_ADRESS         = 0x0A 
let _addr = 0x30
let outDir = 0
let outEmergencyFlag = 0
let outLeft = 0
let outFront = 0
let outRight = 0


//% block="Matrix LiDAR Distance"
//% weight=100 color=#5b3fe8 icon="\uf124"
//% groups="['Distance measurement', 'Obstacle Avoidance']"
namespace matrixLidarDistance {

    export enum ObstacleSide {
        //% block="Left"
        Left = 1,
        //% block="Front"
        Front = 2,
        //% block="Right"
        Right = 3,
    }

    export enum Addr{
        //% block="0x33"
        Addr4 = 0x33,
        //% block="0x32"
        Addr3 = 0x32,
        //% block="0x31"
        Addr2 = 0x31,
        //% block="0x30"
        Addr1 = 0x30,
    }
    
    export enum Matrix {
        //% block="Obstacle avoidance"
        OBS = 4,
        //% block="Matrix"
        MAT = 8,
    }


    /**
     * Initialize the I2C configuration and matrix mode configuration of the sensor. 
     * In the obstacle avoidance mode, only the 4*4 mode is supported in order to increase the frame rate.
     */
    //% block="Initialize the I2C address $address of the laserranging sensor in $matrix matrix mode"
    //% address.defl=Addr.Addr1
    //% weight=97
    export function initialize(address: Addr, matrix: Matrix ):void{
        _addr = address
        let length = 4
        let sendBuffer = pins.createBuffer(8)
        sendBuffer[0] = 0x55
        sendBuffer[1] = ((length + 1) >> 8) & 0xFF
        sendBuffer[2] = (length + 1) & 0xFF
        sendBuffer[3] = CMD_SETMODE
        switch (matrix){
            case Matrix.OBS:
                sendBuffer[4] = 1
            break
            case Matrix.MAT:
                sendBuffer[4] = 0
            break
            default:
            break
        }
       
        sendBuffer[5] = 0
        sendBuffer[6] = 0 
        sendBuffer[7] = matrix
        pins.i2cWriteBuffer(_addr, sendBuffer)
        basic.pause(10)//10 ms
        let buf = recvPacket(_addr, CMD_CONFIG_AVOID)
        if (buf[0] == ERR_CODE_NONE || buf[0] == STATUS_SUCCESS) {
            let len = buf[2] << 8 | buf[3]
        }

    }

    /**
     * Acquire the obstacle avoidance related data that has undergone 
     * comprehensive internal calculations once. The matrix mode needs 
     * to be set to the 4*4 mode. The 8*8 mode will cause data errors or no data.
     */
    //% block="Acquire obstacle avoidance data"
    //% weight=95
    export function getData():void{
        let length = 0
        let sendBuffer = pins.createBuffer(4);
        sendBuffer[0] = 0x55
        sendBuffer[1] = ((length + 1) >> 8) & 0xFF
        sendBuffer[2] = (length + 1) & 0xFF
        sendBuffer[3] = CMD_AVOID_OBSTACLE
        pins.i2cWriteBuffer(_addr, sendBuffer)
        basic.pause(10)//10 ms
        let buf = recvPacket(_addr, CMD_AVOID_OBSTACLE)
        if (buf[0] == ERR_CODE_NONE || buf[0] == STATUS_SUCCESS) {
            outDir = buf[4]
            outEmergencyFlag = buf[5]
            outLeft = buf[6] | buf[7] << 8
            outFront = buf[8] | buf[9] << 8
            outRight = buf[10] | buf[11] << 8
        }
    }
    
    /**
     * Customize the distance value of the obstacle that triggers the 
     * "direction indication". Since the laser sensor has a detection range of 60 degrees and some 
     * laser beams may be directed towards the ground, when it is used on MaqueenPlus, 
     * the maximum set distance should not exceed 200 mm, otherwise the ground may be detected, 
     * resulting in incorrect data. It is also possible to avoid the laser beams hitting the ground by 
     * increasing the installation distance.
     * @param distance to distance ,eg: 200
     */

    //% block="Customize the obstacle avoidance distance to $distance (mm)"
    //% distance.min=100 distance.max=500 distance.defl=200
    //% weight=90
    export function setObstacleDistance(distance: number):void{
        let length = 2
        let _wall = distance
        let sendBuffer = pins.createBuffer(6);
        sendBuffer[0] = 0x55
        sendBuffer[1] = ((length + 1) >> 8) & 0xFF
        sendBuffer[2] = (length + 1) & 0xFF
        sendBuffer[3] = CMD_CONFIG_AVOID
        sendBuffer[4] = (_wall >> 8) & 0xff
        sendBuffer[5] = _wall & 0xff
        pins.i2cWriteBuffer(_addr,sendBuffer)
        basic.pause(10)//10 ms
        let buf = recvPacket(_addr, CMD_CONFIG_AVOID)
        if (buf[0] == ERR_CODE_NONE || buf[0] == STATUS_SUCCESS){
            let len = buf[2] << 8 | buf[3]
        }
    }

    /**
     * When an obstacle within 100 mm is detected, it will output 1; otherwise, it will output 0.
     */
    /*
    //% block="Warn for close range within 100 mm"
    //% weight=80
    export function emergencyWarning(): number {
        return outEmergencyFlag
    }*/

    /**
     * According to the settings of "customized obstacle avoidance distance", 
     * when an obstacle is detected in front, prompt the possible passing directions. 
     * Return values: 1 for the left side, 2 for the right side, 3 for the front.
    */

    //% block="Direction indication in obstacle avoidance mode"
    //% weight=70
    export function obstacleSuggestion(): number {
        return outDir
    }

    /**
     * The black is operating in the obstacle avoidance mode. 
     * The data has undergone comprehensive calculations inside on the matrix laser data. 
     * It can detect the obstacle situations in the left, middle and right directions, 
     * which is convenient for flexible applications.  
     * The matrix mode needs to be set to the 4*4 mode. 
     * The 8*8 mode will cause data errors or no data.
     * @param side to side ,eg: ObstacleSide.Left
    */

    //% block="Distance detection in obstacle avoidance mode %side (mm)"
    //% weight=60
    export function getObstacleDistance(side: ObstacleSide): number {
        /*
        let length = 0
        
        let sendBuffer = pins.createBuffer(4);
        sendBuffer[0] = 0x55
        sendBuffer[1] = ((length + 1) >> 8) & 0xFF
        sendBuffer[2] = (length + 1) & 0xFF
        sendBuffer[3] = CMD_OBSTACLE_DISTANCE
        pins.i2cWriteBuffer(_addr, sendBuffer)
        basic.pause(10)//10 ms
        let buf = recvPacket(CMD_OBSTACLE_DISTANCE)
        if (buf[0] == ERR_CODE_NONE || buf[0] == STATUS_SUCCESS) {
            switch (side){
                case ObstacleSide.Left:
                    ret = buf[4] | buf[5] << 8
                    break
                case ObstacleSide.Front:
                    ret = buf[6] | buf[7] << 8
                    break
                case ObstacleSide.Right:
                    ret = buf[8] | buf[9] << 8
                    break
                default:
                    break

            }
        }*/
        let ret = 0
        switch (side) {
            case ObstacleSide.Left:
                ret = outLeft
                break
            case ObstacleSide.Front:
                ret = outFront
                break
            case ObstacleSide.Right:
                ret = outRight
                break
            default:
                break
        }
        return ret
    }

    /**
     * The black is an independent one working in the matrix mode. It supports both 4*4 and 8*8 modes. 
     * It can read the distance values detected by any of the 64 laser beams.
     * @param x to x ,eg: 3
     * @param y to y ,eg: 3
     */

    //% block="$address Measure the distance of the specified point in matrix mode x: %x y: %y mm"
    //% weight=50
    //% address.defl=Addr.Addr1
    //% x.min=0 x.max=7 x.defl=3
    //% y.min=0 y.max=7 y.defl=3
    export function matrixPointOutput(address: Addr, x: number, y: number): number {
        let length = 2
        let ret = 0
        let sendBuffer = pins.createBuffer(6);
        sendBuffer[0] = 0x55
        sendBuffer[1] = ((length + 1) >> 8) & 0xFF
        sendBuffer[2] = (length + 1) & 0xFF
        sendBuffer[3] = CMD_FIXED_POINT
        sendBuffer[4] = x
        sendBuffer[5] = y
        pins.i2cWriteBuffer(address, sendBuffer)
        basic.pause(10)//10 ms
        let buf = recvPacket(address, CMD_FIXED_POINT)
        if (buf[0] == ERR_CODE_NONE || buf[0] == STATUS_SUCCESS) {
            ret = buf[4] | buf[5] << 8
        }
        return ret
    }

    function recvPacket(addr: number,cmd: number): Buffer{
        let sendBuffer = pins.createBuffer(20);
        let time = control.millis()
        while (control.millis() - time < DEBUG_TIMEOUT_MS)
        {
            let status = pins.i2cReadNumber(addr, NumberFormat.Int8LE)
            if (status != 0xff){
                if (status == STATUS_SUCCESS || status == STATUS_FAILED){
                    sendBuffer[0] = status
                    let command = pins.i2cReadNumber(addr, NumberFormat.Int8LE)
                    sendBuffer[1] = command
                    //serial.writeValue("cmd", command)
                    if (command != cmd){
                        return sendBuffer
                    }
                    let lenBuf = pins.i2cReadBuffer(addr,2)
                    let len  = lenBuf[1] << 8 | lenBuf[0]
                    //serial.writeValue("len", len)
                    sendBuffer[2] = lenBuf[0]
                    sendBuffer[3] = lenBuf[1]
                    if(len == 0){
                        return sendBuffer
                    }
                    let dataBuf = pins.i2cReadBuffer(addr, len)
                    for(let i = 0;i < len;i++){
                        sendBuffer[4+i] = dataBuf[i]
                    }
                    return sendBuffer

                }
            }
        }
        return sendBuffer
    }
}

