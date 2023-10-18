devicesRouter.get('/api/positions', (req, res) => {
  console.log('positionsget');
  res.json(Object.values(data.state.positions));
});
