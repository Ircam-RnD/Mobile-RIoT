export function getScale(domain, range) {
  const a = (range[1] - range[0]) / (domain[1] - domain[0]);
  const b = range[0] - domain[0] * a;

  const min = Math.min.apply(null, domain);
  const max = Math.max.apply(null, domain);

  return x => {
    x = Math.min(max, Math.max(min, x)); // clip
    return a * x + b;
  }
}
