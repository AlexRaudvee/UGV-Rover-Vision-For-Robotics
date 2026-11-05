const defaultConfigModule = await import(`${base_url}/default_widget_config`);
const default_config = defaultConfigModule.default;

export function saveJsonToFile(data, filename) {
	const jsonData = JSON.stringify(data, null, 2);
	const blob = new Blob([jsonData], { type: 'application/json' });
	const url = URL.createObjectURL(blob);
  
	const a = document.createElement('a');
	a.href = url;
	a.download = filename;
	a.click();
  
	URL.revokeObjectURL(url);
}

export class Settings {

	constructor() {
		if (localStorage.hasOwnProperty("settings")) {
			this.fromJSON(localStorage.getItem("settings"));
		}else{
			this.fromJSON(default_config);
		}
		// Add newly supplied perception views to an existing saved dashboard.
		// Keep the user's other widgets and each existing view's positioning.
		const defaults = JSON.parse(default_config);
		if (defaults.layout_version && this.layout_version !== defaults.layout_version) {
			this.navbar = this.navbar || [];
			for (const id of defaults.managed_widgets || []) {
				if (!this[id]) this[id] = { ...defaults[id] };
				if (!this.navbar.some(widget => widget.id === id)) {
					const widget = defaults.navbar.find(widget => widget.id === id);
					if (widget) this.navbar.push({ ...widget });
				}
			}
			for (const widget of this.navbar) {
				const config = this[widget.id];
				if (config && config.topic === '/obstacle_detection/histogram/compressed') {
					config.topic = '/polar_hist/obstacle/histogram/compressed';
				}
			}
			this.layout_version = defaults.layout_version;
			this.save();
		}
	}

	fromJSON(settings_object){
		let storedSettings = JSON.parse(settings_object);
		Object.assign(this, storedSettings);
	}

	save() {
		localStorage.setItem("settings", JSON.stringify(this));
	}
}

export let settings = new Settings();
