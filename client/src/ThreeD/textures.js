import groundTextureImg from '../resources/3d/ground.jpg';
import { NearestFilter, RepeatWrapping, TextureLoader } from 'three'


const groundTexture = new TextureLoader().load(groundTextureImg);
groundTexture.wrapS = groundTexture.wrapT = RepeatWrapping;

export { groundTexture };

