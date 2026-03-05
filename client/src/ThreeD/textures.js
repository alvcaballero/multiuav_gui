import groundTextureImg from '../resources/3d/ground.jpg';
import { NearestFilter, RepeatWrapping, TextureLoader, CanvasTexture } from 'three';

const groundTexture = new TextureLoader().load(groundTextureImg);
groundTexture.wrapS = groundTexture.wrapT = RepeatWrapping;

// Textura de agua estilo Minecraft generada proceduralmente
const createWaterTexture = () => {
  const size = 64;
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext('2d');

  // Paleta estilo Minecraft agua
  const colors = ['#1a6ba0', '#1d77b2', '#2185c5', '#1e7ab8', '#1670a8', '#2490d0', '#1c7dc0'];

  const blockSize = 8; // bloques de 8x8 px
  for (let y = 0; y < size; y += blockSize) {
    for (let x = 0; x < size; x += blockSize) {
      const color = colors[Math.floor(Math.random() * colors.length)];
      ctx.fillStyle = color;
      ctx.fillRect(x, y, blockSize, blockSize);

      // borde más oscuro para efecto pixel
      ctx.fillStyle = 'rgba(0,0,0,0.15)';
      ctx.fillRect(x, y, blockSize, 1);
      ctx.fillRect(x, y, 1, blockSize);

      // brillo superior izquierdo
      ctx.fillStyle = 'rgba(255,255,255,0.08)';
      ctx.fillRect(x + 1, y + 1, blockSize - 2, 2);
    }
  }

  const texture = new CanvasTexture(canvas);
  texture.magFilter = NearestFilter; // pixelado estilo Minecraft
  texture.wrapS = texture.wrapT = RepeatWrapping;
  return texture;
};

export const waterTexture = createWaterTexture();
export { groundTexture };

