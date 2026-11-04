import {defineConfig} from 'vite';
export default defineConfig({server:{proxy:{'/api':{target:'http://127.0.0.1:8765',ws:true},'/assets3d':'http://127.0.0.1:8765'}},build:{chunkSizeWarningLimit:1000}});
