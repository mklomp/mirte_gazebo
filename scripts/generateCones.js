
const yaml = require('js-yaml');
const fs = require('fs')
function coneDistance(x) {
    return 0.1
}

function places(x) {
    // return Math.cos(x-2)+2
    // return 0.2083 *x*x*x - 2.125*x*x + 6.4167*x - 4
    // return 0.0333*x*x*x - 0.5*x*x + 1.8667*x + 1
    // return 0.1083*x*x*x - 1.175*x*x + 3.5167*x - 1.2
    return (x-6.8)*4+1

}

function streetWidth(x) {
    return 0
}

maxX = 7.8
minX = 6.8


currX = minX

cones = []
while(currX < maxX) {
    y = places(currX)
    yLeft = y-streetWidth(currX)/2
    yRight = y+streetWidth(currX)/2
    cones.push({kind: 'cone',
        pos: [currX, yLeft],
        rotate: 0,
        height: 0.1
      })
    //   cones.push({kind: 'cone',
    //   pos: [currX, yRight],
    //   rotate: 0,
    //   height: 0.1
    // })



    currX+=coneDistance(currX)
}

console.log(yaml.dump(cones))
fs.writeFileSync('./cones.tmp', yaml.dump(cones), {encoding:'utf-8'})