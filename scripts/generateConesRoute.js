
const yaml = require('js-yaml');
const fs = require('fs')
const path = require('path')

const map = yaml.load(fs.readFileSync(path.join(__dirname, '../maps/routeE.yaml')))

// console.log(map);

cones = []
function buildStraight(x, y, direction) {
    if (direction == "N" || direction == "S") {
        cones.push(...[
            { x, y }, { y: y + 0.25, x }, { y: y + 0.5, x }, { y: y + 0.75, x }, { y: y + 1, x }
        ])
        x += 1;
        cones.push(...[
            { x, y }, { y: y + 0.25, x }, { y: y + 0.5, x }, { y: y + 0.75, x }, { y: y + 1, x }
        ])
    } else { // east/west
        cones.push(...[
            { x, y }, { x: x + 0.25, y }, { x: x + 0.75, y }, { x: x + 0.5, y }, { x: x + 1, y }
        ])
        y += 1;
        cones.push(...[
            { x, y }, { x: x + 0.25, y }, { x: x + 0.75, y }, { x: x + 0.5, y }, { x: x + 1, y }
        ])
    }

}

function buildCorner(x, y, direction) {
    additions = [
        [0, 1], [1, 0], [0.05, 0.75], [0.75, 0.05], [0.2,0.2], [0.5, 0.1], [0.1, 0.5]
    ]
    if (direction == "W") {
        console.log('corner w')
        cones.push(...additions.map(it => { return { x: x + it[0], y: y + it[1] } }));
    }
    if (direction == "E") {
        console.log('corner e')
        cones.push(...additions.map(it => { return { x: x+1 - it[0], y: y+1 - it[1] } }));
    }
    if (direction == "N") {
        console.log('corner n')
        cones.push(...additions.map(it => { return { x: x+1 - it[1], y: y + it[0] } }));
    }
    if (direction == "S") {
        console.log('corner s')
        cones.push(...additions.map(it => { return { x: x + it[1], y: y+1 - it[0] } }));
    }
}


map.tiles.forEach((row, y) => {
    row.forEach((item, x) => {
        if (item.startsWith('straight')) {
            buildStraight(x, y, item.split('/')[1])
        }
        if (item.startsWith('curve_left')) {
            buildCorner(x, y, item.split('/')[1])
        }
        if(item.startsWith('curve_right')) {
            dirs = ['N', "E", "S", 'W'];
            buildCorner(x, y,dirs[(dirs.indexOf( item.split('/')[1])+3) %4])
        }
    })
});
map.objects.push(...cones.map(it => {
    return { kind: 'cone', pos: [it.x, it.y], rotate: 0, height: 0.1 }}))
fs.writeFileSync(path.join(__dirname,'../maps/route.yaml'), yaml.dump(map))