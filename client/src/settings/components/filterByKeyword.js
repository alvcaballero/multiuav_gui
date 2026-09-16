export const filterByKeyword = (keyword) => (item) =>
  !keyword || JSON.stringify(item).toLowerCase().includes(keyword.toLowerCase());
