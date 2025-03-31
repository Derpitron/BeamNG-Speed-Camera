angular.module('beamng.apps')
.directive('derpSosSettings', [function () {
	return {
		templateUrl: '/ui/modules/apps/derpSosSettings/app.html',
		replace: true,
		restrict: 'EA',
		link: function (scope, element, attrs) {
		    scope.showDiv = true;
			
			bngApi.engineLua('settings.getValue("camPosX")', function (data) {
		      scope.$evalAsync(function () {
		        scope.camPosX = data;
		      });
		    });

		    bngApi.engineLua('settings.getValue("camPosY")', function (data) {
		      scope.$evalAsync(function () {
		        scope.camPosY = data;
		      });
		    });

			bngApi.engineLua('settings.getValue("camPosZ")', function (data) {
				scope.$evalAsync(function () {
				  scope.camPosZ = data;
				});
			  });

			  bngApi.engineLua('settings.getValue("camRotX")', function (data) {
				scope.$evalAsync(function () {
				  scope.camRotX = data;
				});
			  });

			  bngApi.engineLua('settings.getValue("camRotY")', function (data) {
				scope.$evalAsync(function () {
				  scope.camRotY = data;
				});
			  });

			  bngApi.engineLua('settings.getValue("camRotZ")', function (data) {
				scope.$evalAsync(function () {
				  scope.camRotZ = data;
				});
			  });

		    

			scope.$on('$destroy', function () {
			});


			scope.$watch('camPosX', function (newVal, oldVal) {
				if(newVal !== undefined) {
					bngApi.engineLua('settings.setValue("camPosX",'+newVal+')');
				}
			});

			scope.$watch('camPosY', function (newVal, oldVal) {
				if(newVal !== undefined) {
					bngApi.engineLua('settings.setValue("camPosY",'+newVal+')');
				}
			});

			scope.$watch('camPosZ', function (newVal, oldVal) {
				if(newVal !== undefined) {
					bngApi.engineLua('settings.setValue("camPosZ",'+newVal+')');
				}
			});

			scope.$watch('camRotX', function (newVal, oldVal) {
				if(newVal !== undefined) {
					bngApi.engineLua('settings.setValue("camRotX",'+newVal+')');
				}
			});

			scope.$watch('camRotY', function (newVal, oldVal) {
				if(newVal !== undefined) {
					bngApi.engineLua('settings.setValue("camRotY",'+newVal+')');
				}
			});

			scope.$watch('camRotZ', function (newVal, oldVal) {
				if(newVal !== undefined) {
					bngApi.engineLua('settings.setValue("camRotZ",'+newVal+')');
				}
			});

			

			scope.resetSettings = function(){
				bngApi.engineLua('settings.setValue("camPosX", 0)');
				bngApi.engineLua('settings.setValue("camPosY", 0)');
				bngApi.engineLua('settings.setValue("camPosZ", 0)');
                bngApi.engineLua('settings.setValue("camRotX", 7.5)');
				bngApi.engineLua('settings.setValue("camRotY", 1)');
				bngApi.engineLua('settings.setValue("camRotZ", 0)');
				scope.camPosX = 0;
				scope.camPosY = 0;
				scope.camPosZ = 0;
				scope.camRotX = 7.5;
				scope.camRotY = 1;
				scope.camRotZ = 0;
			};

			scope.showHide = function() {
				scope.showDiv = !scope.showDiv;
			};
		}
	};
}])
