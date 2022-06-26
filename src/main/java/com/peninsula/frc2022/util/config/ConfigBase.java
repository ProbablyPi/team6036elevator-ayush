package com.peninsula.frc2022.util.config;

public abstract class ConfigBase {

	@Override
	public String toString() {
		return Configs.toJson(this);
	}

	void onPostUpdate() {
	}
}
