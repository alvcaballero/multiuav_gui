/**
 * HTML template for webapp build not found error page
 * @param {Object} config - Server configuration
 * @returns {string} HTML string
 */
export const getErrorPageHTML = ({ port, nodeVersion, rosEnabled, fbEnabled, llmEnabled }) => `
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>MultiUAV GUI - Error</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      display: flex;
      justify-content: center;
      align-items: center;
      min-height: 100vh;
      padding: 20px;
    }
    .error-container {
      background: white;
      border-radius: 12px;
      box-shadow: 0 20px 60px rgba(0,0,0,0.3);
      padding: 40px;
      max-width: 600px;
      text-align: center;
    }
    .error-icon {
      font-size: 64px;
      margin-bottom: 20px;
    }
    h1 {
      color: #2d3748;
      font-size: 32px;
      margin-bottom: 16px;
    }
    p {
      color: #4a5568;
      font-size: 16px;
      line-height: 1.6;
      margin-bottom: 12px;
    }
    .error-code {
      background: #fed7d7;
      color: #c53030;
      padding: 8px 16px;
      border-radius: 6px;
      font-family: 'Courier New', monospace;
      font-size: 14px;
      margin: 20px 0;
      display: inline-block;
    }
    .instructions {
      background: #f7fafc;
      border-left: 4px solid #4299e1;
      padding: 20px;
      margin-top: 24px;
      text-align: left;
      border-radius: 6px;
    }
    .instructions h2 {
      color: #2d3748;
      font-size: 18px;
      margin-bottom: 12px;
    }
    .instructions code {
      background: #2d3748;
      color: #48bb78;
      padding: 2px 8px;
      border-radius: 4px;
      font-family: 'Courier New', monospace;
      font-size: 14px;
    }
    .instructions ol {
      margin-left: 20px;
      margin-top: 12px;
    }
    .instructions li {
      margin-bottom: 8px;
      color: #4a5568;
    }
    .server-info {
      margin-top: 24px;
      padding-top: 24px;
      border-top: 1px solid #e2e8f0;
      font-size: 14px;
      color: #718096;
    }
  </style>
</head>
<body>
  <div class="error-container">
    <div class="error-icon">⚠️</div>
    <h1>Cliente Web No Encontrado</h1>
    <p>El servidor de MultiUAV GUI está funcionando, pero la aplicación cliente no ha sido compilada.</p>
    <div class="error-code">Error 503 - Service Unavailable</div>

    <div class="instructions">
      <h2>📋 Pasos para solucionar:</h2>
      <ol>
        <li>Navega al directorio del cliente: <code>cd client</code></li>
        <li>Instala las dependencias (si no lo has hecho): <code>npm install</code></li>
        <li>Compila la aplicación: <code>npm run build</code></li>
        <li>Reinicia el servidor: <code>cd ../server && npm run server</code></li>
      </ol>
    </div>

    <div class="server-info">
      <strong>Servidor MultiUAV GUI</strong><br>
      Puerto: ${port} | Node.js ${nodeVersion}<br>
      ROS: ${rosEnabled ? 'Habilitado' : 'Deshabilitado'} |
      FlatBuffer: ${fbEnabled ? 'Habilitado' : 'Deshabilitado'} |
      LLM: ${llmEnabled ? 'Habilitado' : 'Deshabilitado'}
    </div>
  </div>
</body>
</html>
`;
