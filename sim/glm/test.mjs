// Extracts the SOLVER section from index.html and runs the built-in analytic tests headless.
import {readFileSync} from 'node:fs';

const html=readFileSync(new URL('./index.html',import.meta.url),'utf8');
const m=html.match(/\/\/ ===================== SOLVER BEGIN[^\n]*\n([\s\S]*?)\/\/ ===================== SOLVER END/);
if(!m){ console.error('SOLVER section not found'); process.exit(2); }
const mod=await import('data:text/javascript;base64,'+Buffer.from(m[1]).toString('base64'));

const results=mod.runTests(console.log);
const fails=results.filter(r=>!r.pass);
console.log(`\n${results.length-fails.length}/${results.length} passed`);
process.exit(fails.length?1:0);
