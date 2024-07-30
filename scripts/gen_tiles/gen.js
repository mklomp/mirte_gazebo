const fs = require('fs')
const path = require('path')


const tiles = ["straight", "big_gap", "circle_corner", "crossing", "empty", "floor", "roundabout_corner", "roundabout_square", "small_gap", "square_corner", "straight"]



// for every tile, copy the gen_tile folder to materials folder, and rename the folder to the tile name
tiles.forEach(tile => {
    convert(tile, path.join("/home/arendjan/ros2/ros2_rsp/src/mirte-gazebo/media/materials/textures/RCJ", `${tile}.png`))
});
fs.readdirSync("/home/arendjan/ros2/ros2_rsp/src/mirte-gazebo/media/materials/textures").forEach(file => {
    console.log(file)
    if (!file.endsWith(".png")) {
        return;

    }
    convert(file.replace(".png", ""), path.join("/home/arendjan/ros2/ros2_rsp/src/mirte-gazebo/media/materials/textures", `${file}`))
});    

function convert(name, image) {
    console.log("name:",name, image)
    const src = path.join(__dirname, "tile_empty")
    // const dest = path.join( "/home/arendjan/ros2/ros2_rsp/src/mirte-gazebo/models/", tile)
    const dest = path.join( "/tmp/models/", name)

    fs.mkdirSync(dest, {recursive:true})
    fs.mkdirSync(path.join(dest, "meshes"), {recursive:true})
    fs.readdirSync(src, {recursive:true}).forEach(file => {
        try {
            if(!file.includes(".")) {
                return;
            }
           const filed = file.replace("tile", name)
            console.log("file",file)
            console.log(path.join(dest, file))
            fs.writeFileSync(path.join(dest, filed), fs.readFileSync(path.join(src, file), {encoding: 'utf8'}).replace(/curve\.png/g, `${name}.png`).replace(/tile/g, name), {encoding: 'utf8'})
        // fs.copyFileSync(path.join(src, file), path.join(dest, file))
        } catch (e) {
            console.log(e)
        }
    })
    fs.copyFileSync(image, path.join(dest, `meshes/${name}.png`))

}