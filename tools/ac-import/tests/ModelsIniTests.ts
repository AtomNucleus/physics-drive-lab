import { parseModelsIni, hasExternalModelTransform } from '../ModelsIni';

function assert(condition: unknown, message: string): asserts condition { if (!condition) throw new Error(message); }

const models = parseModelsIni(`
[MODEL_0]
FILE=pk_akina.kn5
POSITION=0,0,0
ROTATION=0,0,0

[MODEL_1]
FILE=physics6.kn5
POSITION=0,0,0
ROTATION=0,0,0
`, '/tracks/pk_akina');

assert(models.length === 2, 'layout model count');
assert(models[0].file === 'pk_akina.kn5', 'first model filename');
assert(models[0].path.endsWith('/tracks/pk_akina/pk_akina.kn5'), 'resolved model path');
assert(models[1].file === 'physics6.kn5', 'physics model filename');
assert(hasExternalModelTransform(models[0]) === false, 'zero external transform');

const transformed = parseModelsIni(`
[MODEL_0]
FILE=offset.kn5
POSITION=10,0,-5
ROTATION=0,5,0
`, '/tracks/test')[0];
assert(hasExternalModelTransform(transformed) === true, 'non-zero external transform should be detected');
console.log('AC models.ini tests: PASS');
