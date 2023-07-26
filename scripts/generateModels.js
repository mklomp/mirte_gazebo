var fs = require('fs');
const fse = require('fs-extra');
const path = require('path');


// To copy a folder or file, select overwrite accordingly
var textureFiles = fs.readdirSync('/home/arendjan/gym-duckietown/gym_duckietown/textures');

var files = fs.readdirSync('/home/arendjan/gym-duckietown/gym_duckietown/meshes');
console.log(files)

let items = files.filter(v => v.endsWith('.obj')).filter(v => !v.includes("barrier") && !v.includes("duckie."));
console.log(items);
let states = [];
let worlds = [];
items.map(v => v.replace(".obj", '')).forEach(async v => {

    const srcDir = `/home/arendjan/catkin_ws/src/mirte-gazebo/models/barrier_orig`;
    const destDir = `/home/arendjan/catkin_ws/src/mirte-gazebo/models/${v}`;



    let rewriteF = async (file) => {
        let filename = `${destDir}/${file}`
        let sdf = await fse.readFile(filename, { encoding: 'utf-8' });
        await fse.writeFile(filename, sdf.replaceAll('barrier', v).replaceAll('Barrier', v), { encoding: 'utf-8' });

    }
    await fse.copy(srcDir, destDir, { overwrite: true });
    await rewriteF('model.sdf');
    await rewriteF('model.config');
    await rewriteF('materials/scripts/barrier.material');
    await fse.move(`${destDir}/materials/scripts/barrier.material`, `${destDir}/materials/scripts/${v}.material`, { overwrite: true })
    await fse.copy(`/home/arendjan/gym-duckietown/gym_duckietown/meshes/${v}.obj`, `${destDir}/meshes/${v}.obj`)
    await fse.remove(`${destDir}/meshes/barrier.obj`)
    await fse.remove(`${destDir}/materials/textures/barrier.png`)
    let text = textureFiles.filter(t => t.includes(v))
    if (text.length > 0) {
        await fse.copy(`/home/arendjan/gym-duckietown/gym_duckietown/textures/${text[0]}`, `${destDir}/materials/textures/${v}.png`)
    }

    let mtls = files.filter(t => t.endsWith('.mtl')).filter(t => t.includes(v));
    if (mtls.length > 0) {
        let mtlF = `${destDir}/meshes/${v}.mtl`;
        await fse.copy(`/home/arendjan/gym-duckietown/gym_duckietown/meshes/${mtls[0]}`, mtlF)
        // await fse.remove(`${destDir}/materials/textures/${v}.mtl`)
        let obj = await fse.readFile(`${destDir}/meshes/${v}.obj`, { encoding: "utf-8" })
        obj = obj.replace(/mtllib.*/, `mtllib ${v}.mtl`)
        await fse.writeFile(`${destDir}/meshes/${v}.obj`, obj, { encoding: "utf-8" })
        let mtl = await fse.readFile(`${destDir}/meshes/${v}.mtl`, { encoding: "utf-8" })

        // console.log([...mtl.matchAll(/textures/g)]);

        let mat = [...mtl.matchAll(/\.\.\/textures\/.*\.png/g), ...mtl.matchAll(/\.\.\/textures\/.*\.jpg/g)];
        mat.forEach(async (textureFile) => {
            console.log(v, 'requres', textureFile[0])
            let tfN = path.basename(textureFile[0])
            await fse.copyFile(`/home/arendjan/gym-duckietown/gym_duckietown/textures/${textureFile[0]}`, `${destDir}/materials/textures/${tfN}`)
        });
        // if(v == "sign_stop"){
        //     console.log("stop",v)
        //     process.exit();
        // }
    }
    states.push(`    <xacro:duckieModel name="name_${v}" type="${v}" size="0.6" x="${states.length * 2}" y="${states.length * 2}" yaw="0"/>`)
    worlds.push(`      <xacro:duckieState name="name_${v}" type="${v}" size="0.6" x="${states.length * 2}" y="${states.length * 2}"  yaw="0"/>        `)
    //       <xacro:duckieState name="barr" type="barrier" size="0.6" x="1" y="8" yaw="0"/>

    await scaleObject(v);
    console.log("done ", v);

})

const OBJFile = require('obj-file-parser');
let scales = [];
async function scaleObject(name) {
    const fileDir = `/home/arendjan/catkin_ws/src/mirte-gazebo/models/${name}`;
    let obj = await fse.readFile(`${fileDir}/meshes/${name}.obj`, { encoding: 'utf-8' })
    const objFile = new OBJFile(obj).parse();
    // console.log(objFile);
    let maxX = objFile.models.reduce((prevX, currV)=>currV.vertices.reduce((prev, curr) => {
        // console.log(curr)
        return Math.max(curr.y, prev)
    }, prevX), -1000);
    let minX = objFile.models.reduce((prevX, currV)=>currV.vertices.reduce((prev, curr) => {
        // console.log(curr)
        return Math.min(curr.y, prev)
    }, prevX),1000);
    let scale = 1 / (maxX - minX);
    let sdf = await fse.readFile(`${fileDir}/model.sdf`, { encoding: 'utf-8' });
    sdf = sdf.replaceAll('<scale>1 1 1</scale>', `<scale>${scale} ${scale} ${scale}</scale>`);
    await fse.writeFile(`${fileDir}/model.sdf`, sdf, { encoding: 'utf-8' })
    scales.push(`  <xacro:if value="\${type == '${name}'}">
    <xacro:property name="model_scale" value="\${${scale}}"/>
    <xacro:property name="z_offset" value="\${${-minX}}"/>
    </xacro:if>`)
}



setTimeout(async () => {
    await scaleObject('duckie');
    await scaleObject('barrier');
    // console.log(states.join('\n'));
    // console.log(worlds.join('\n'));
    console.log(scales.join('\n'));
}, 5000);