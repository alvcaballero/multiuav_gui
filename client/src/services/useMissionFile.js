import { useDispatch } from 'react-redux';
import { missionActions } from '../store';
import { readTextFile, parseMissionFile } from './fileService';

/**
 * Hook que encapsula la lectura y carga de un archivo de misión al store.
 * Elimina el boilerplate de FileReader + parseo + dispatch de los componentes.
 *
 * @returns {(file: File) => void}
 */
export const useMissionFile = () => {
  const dispatch = useDispatch();

  return (file) => {
    readTextFile(file, ({ name, data }) => {
      const result = parseMissionFile({ name, data });
      if (!result) {
        alert('Formato de archivo no soportado');
        return;
      }
      dispatch(missionActions.updateMission({ ...result.mission, name: result.name }));
    });
  };
};
