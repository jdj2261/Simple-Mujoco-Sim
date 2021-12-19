import os
import trimesh

from lxml import etree
from itertools import zip_longest

from logger import logger

def convexify_mesh(mesh_names, mesh_directory, mesh_units, mesh_type='stl'):
    for name, mesh_dir, unit in zip_longest(mesh_names, mesh_directory, mesh_units):
        mesh_file = mesh_dir + 'meshes/' + name + '.' + mesh_type
        mesh = trimesh.load(mesh_file)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate((trimesh.Trimesh(
                vertices=g.vertices, faces=g.faces) for g in mesh.geometry.values()))
        
        mesh.apply_translation(-mesh.center_mass)

        try:
            convex_list = mesh.convex_decomposition(maxhulls=20, resolution=8000000, oclAcceleration=1)
        except (FileNotFoundError, ValueError) as e:
            logger.error(e)
            logger.error("Download testVHACD in https://github.com/kmammou/v-hacd.git")
            logger.error("Copy testVHACD to /usr/local/bin")

        if not isinstance(convex_list, trimesh.base.Trimesh):
            logger.info(f"The length of Convex List is {len(convex_list)}")
            for i in range(len(convex_list)):
                new_name = mesh_dir + 'meshes/' + name + f'_{str(i)}' '.' + mesh_type
                part_trimesh = convex_list[i]
                if not os.path.exists(os.path.dirname(new_name)):
                    os.makedirs(os.path.dirname(new_name))
                trimesh.exchange.export.export_mesh(part_trimesh, new_name, file_type="stl")
        else:
            logger.info('No decomposition')
            new_name = mesh_dir + 'meshes/' + name + '.' + mesh_type
            if not os.path.exists(os.path.dirname(new_name)):
                os.makedirs(os.path.dirname(new_name))
            trimesh.exchange.export.export_mesh(mesh, new_name, file_type="stl")

        mujoco = etree.Element('mujoco', model=name)
        asset = etree.Element('asset')
        worldbody = etree.Element('worldbody')
        body = etree.Element('body', name=name)
        body_col = etree.Element('body', name='collision')
        body_vis = etree.Element('body', name='visual')
        site1 = etree.Element('site', rgba='0 0 0 0', size='0.005', pos='0 0 -0.06', name='bottom_site')
        site2 = etree.Element('site', rgba='0 0 0 0', size='0.005', pos='0 0 0.04', name='top_site')
        site3 = etree.Element('site', rgba='0 0 0 0', size='0.005', pos='0.025 0.025 0', name='horizontal_radius_site')

        if not isinstance(convex_list, trimesh.base.Trimesh):
            for i in range(len(convex_list)):
                asset_file = 'meshes/' + name + '_' + str(i) + '.' + mesh_type
                asset_name = name + '_' + str(i)
                asset_scale=str(unit)+' '+str(unit)+' '+str(unit)
                asset.append(etree.Element('mesh', file=asset_file, name=asset_name, scale=asset_scale))

                body_col_name = name + '_' + str(i)
                body_col.append(etree.Element('geom', pos='0 0 0', mesh=body_col_name, type='mesh',
                                            density='5000', rgba='0 1 0 1',
                                            group='0', condim='6'))

                body_vis_name = name + '_' + str(i)
                body_vis.append(etree.Element('geom', pos='0 0 0', mesh=body_vis_name, type='mesh', 
                                            rgba='0 1 0 1',conaffinity='0', contype='0', group='1', mass='0.0001'))
        else:
            asset_name = 'meshes/' + name + '.' + mesh_type
            asset_scale = str(unit)+' '+str(unit)+' '+str(unit)
            asset.append(etree.Element('mesh', file=asset_name, name=name, scale=asset_scale))

            body_col.append(etree.Element('geom', pos='0 0 0', mesh=name, type='mesh',
                                        density='5000', rgba='0 1 0 1',
                                        group='0', condim='6'))
            body_vis.append(
                etree.Element('geom', pos='0 0 0', mesh=name, type='mesh', rgba='0 1 0 1',
                            conaffinity='0', contype='0', group='1', mass='0.0001'))

        mujoco.append(asset)
        mujoco.append(worldbody)
        body.append(body_col)
        body.append(body_vis)
        body.append(site1)
        body.append(site2)
        body.append(site3)
        worldbody.append(body)
        s = etree.tostring(mujoco, pretty_print=True)

        with open(mesh_dir + name + '.xml', 'wb') as f:
            f.write(s)
        logger.info(name + ' processing completed')

if __name__ == '__main__':
    mesh_dir = ['/Users/jindaejong/Mywork/Mujoco-Sim/models/assets/objects/']
    convexify_mesh(mesh_names=['box_goal'], mesh_directory=mesh_dir, mesh_units=[0.0013])