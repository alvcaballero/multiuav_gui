import { elementItemsModel } from './elementItems.js';
import { elementGroupsModel } from './elementGroups.js';
import { elementTypesModel } from './elementTypes.js';
import { geodeticToENU } from '../mission/coordinateConverter.js';

/**
 * @typedef {Object} InspectionTarget
 * @property {number} id - ElementItem id (catalog primary key)
 * @property {string} name - Element name
 * @property {string} groupName - Owning ElementGroup name
 * @property {string} type - ElementType id (e.g. 'windTurbine')
 * @property {{x:number,y:number,z:number}|{lat:number,lng:number,alt:number}} position - Local ENU position when globalOrigin is given, raw geodetic otherwise
 * @property {string} description - ElementItem description
 * @property {string} groupdescription - Owning ElementGroup description
 * @property {Object} attributes - ElementItem attributes (e.g. height, rotor_diameter)
 */

/**
 * Resolve the owning ElementGroup + ElementType for a batch of ElementItems
 * in as few queries as possible.
 * @param {Array} items - ElementItem rows
 * @returns {Promise<{groupById: Map, typeById: Map}>}
 */
async function resolveGroupsAndTypes(items) {
  const groupIds = [...new Set(items.map((item) => item.groupId))];
  const groups = await Promise.all(groupIds.map((groupId) => elementGroupsModel.getById(groupId)));
  const groupById = new Map(groups.filter(Boolean).map((group) => [group.id, group]));

  const typeIds = [...new Set([...groupById.values()].map((group) => group.typeId))];
  const types = await Promise.all(typeIds.map((typeId) => elementTypesModel.getById(typeId)));
  const typeById = new Map(types.filter(Boolean).map((type) => [type.id, type]));

  return { groupById, typeById };
}

/**
 * Shape a catalog ElementItem (plus its resolved group/type) into an
 * InspectionTarget. Every field comes from the catalog, never from a caller.
 * @param {Object} item - ElementItem row
 * @param {Map} groupById
 * @param {Map} typeById
 * @param {{lat:number,lng:number,alt:number}} [globalOrigin] - When given, position is local ENU; otherwise raw geodetic {lat,lng,alt}
 * @returns {InspectionTarget}
 */
function itemToInspectionTarget(item, groupById, typeById, globalOrigin) {
  const group = groupById.get(item.groupId);
  const type = group ? typeById.get(group.typeId) : null;
  const groupName = group?.name ?? null;
  const typeId = type?.id ?? group?.typeId ?? null;
  const alt = item.attributes?.altitude ?? item.attributes?.alt ?? 0;
  const position = globalOrigin
    ? geodeticToENU(item.latitude, item.longitude, alt, globalOrigin)
    : { lat: item.latitude, lng: item.longitude, alt };

  return {
    id: item.id,
    name: item.name,
    groupName,
    type: typeId,
    position,
    description: item.description,
    groupdescription: group?.description ?? null,
    attributes: item.attributes,
  };
}

/**
 * Resolve inspection targets against the SQL catalog (ElementItems/
 * ElementGroups/ElementTypes). Every returned field (name, group, type,
 * position, description, attributes) comes from the catalog, never from the
 * caller, so a hallucinated value can't slip through validation as if it
 * were real.
 *
 * `entries` may be plain ids (legacy — trust nothing but the id) or full
 * `{id, name, group, type}` objects. When an entry carries name/group/type,
 * those are cross-checked against the catalog record for that id; a mismatch
 * is reported instead of silently using the LLM's value.
 *
 * @param {(number|string|{id:(number|string), name?:string, group?:string, type?:string})[]} entries
 * @param {{lat:number,lng:number,alt:number}} [globalOrigin] - When given, position is converted to this frame's local ENU; otherwise raw geodetic {lat,lng,alt} is returned
 * @returns {Promise<{targets: InspectionTarget[], notFound: (number|string)[], mismatched: string[]}>}
 */
export async function resolveInspectionTargets(entries, globalOrigin) {
  const requests = entries.map((entry) => (entry !== null && typeof entry === 'object' ? entry : { id: entry }));
  const numericIds = [...new Set(requests.map((request) => Number(request.id)))];
  const items = await elementItemsModel.getByIds(numericIds);
  const itemById = new Map(items.map((item) => [item.id, item]));

  const { groupById, typeById } = await resolveGroupsAndTypes(items);

  const notFound = [];
  const mismatched = [];
  const targets = [];

  for (const request of requests) {
    const item = itemById.get(Number(request.id));
    if (!item) {
      notFound.push(request.id);
      continue;
    }

    const group = groupById.get(item.groupId);
    const groupName = group?.name ?? null;
    const typeId = group ? (typeById.get(group.typeId)?.id ?? group.typeId) : null;

    const wantsCrossCheck = request.name !== undefined || request.group !== undefined || request.type !== undefined;
    if (wantsCrossCheck) {
      const nameOk = request.name === undefined || request.name === item.name;
      const groupOk = request.group === undefined || request.group === groupName;
      const typeOk = request.type === undefined || request.type === typeId;
      if (!nameOk || !groupOk || !typeOk) {
        mismatched.push(
          `Target id=${request.id} mismatch: expected name="${item.name}"/group="${groupName}"/type="${typeId}", ` +
            `got name="${request.name}"/group="${request.group}"/type="${request.type}".`
        );
        continue;
      }
    }

    targets.push(itemToInspectionTarget(item, groupById, typeById, globalOrigin));
  }

  return { targets, notFound, mismatched };
}

/**
 * Resolve every catalog ElementItem inside a geographic bounding box, except
 * the given ids (typically the mission's own targets). Used to surface
 * nearby infrastructure the LLM didn't pick as an inspection target but
 * still needs to know about for collision avoidance — there's no computed
 * geometry for these yet, so the LLM reasons over `attributes`/`description`
 * itself, same as it does for targets.
 * @param {{minLat:number, maxLat:number, minLng:number, maxLng:number}} bounds
 * @param {(number|string)[]} excludeIds
 * @param {{lat:number,lng:number,alt:number}} [globalOrigin] - When given, position is local ENU; otherwise raw geodetic {lat,lng,alt}
 * @returns {Promise<InspectionTarget[]>}
 */
export async function resolveObstaclesInBounds(bounds, excludeIds, globalOrigin) {
  const excludeSet = new Set(excludeIds.map((id) => Number(id)));
  const items = (await elementItemsModel.getAllInBounds(bounds)).filter((item) => !excludeSet.has(item.id));

  const { groupById, typeById } = await resolveGroupsAndTypes(items);

  return items.map((item) => itemToInspectionTarget(item, groupById, typeById, globalOrigin));
}
